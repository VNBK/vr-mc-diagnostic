/*
 * cia402_drive_sim.c - simulated CiA 402 drive node for master testing.
 *
 * A protocol-level test slave: speaks DS-402 over CAN-FD against
 * motor_drive_master (or any external CANopen-FD master) and answers
 * with cleanly-shaped state from a kinematic virtual motor. No FOC,
 * no current loops, no sensor ambiguity — the kind of slave you want
 * sitting on the wire while you develop master-side bring-up,
 * trajectory generation, fault handling, or boot upgrade.
 *
 * For a realistic FOC simulation against a PMSM plant model, see
 * motor_drive_cia402.c — same wire protocol, same OD layout, but
 * driven by the full motor_drive + plant + sensor stack.
 *
 * Wires three pieces together:
 *   vmotor     - pure-kinematic virtual motor (no FOC, no tuning). Slews
 *                position / velocity / torque in response to cia402
 *                targets at 10 kHz.
 *   cia402     - DS-402 drive profile (state machine + 14 mandatory OD
 *                entries).
 *   can_stack  - PDO + USDO server, plus a USDO client for the shell to
 *                act as a remote master.
 *
 * Topology (one process, virtual broadcast bus):
 *
 *   Node 5 ("drive"):  vmotor + cia402 + PDO + USDO server
 *   Node 7 ("master"): USDO client (sdo_get / sdo_put commands)
 *
 * Bridge between cia402 and the virtual motor:
 *   state   OPERATION_ENABLED  ->  vmotor.enabled = true
 *           anything else      ->  vmotor.enabled = false (coast)
 *   mode    csp/pp             ->  position-following slew loop
 *           csv/pv/velocity    ->  velocity ramp loop
 *           cst/pt             ->  torque-injection loop
 *   tick    cia402 target*     ->  vmotor.target_*
 *           vmotor actuals     ->  cia402_set_*_actual
 *
 * Unit conversions (drive <-> motor):
 *   position  int32 increments  ==  float radians * (2^POS_INC_BITS) / (2*pi)
 *   velocity  int32 increments/s  (same factor)
 *   torque    int16 per-thousand-of-rated  ==  float Nm / RATED_TORQUE_NM * 1000
 *
 * Shell provides both local (ergonomic) and remote (SDO) paths so the same
 * binary shows how an app talks to itself and how an external master would
 * talk to it over CANopen.
 */

#include "cia402.h"
#include "can_fd_pdo.h"
#include "co_fd_usdo.h"
#include "co_od.h"

#include "vrmc_sim_od.h"

/* Platform porting umbrella. Pulls in hal.h (so hal_can_t et al.
 * resolve), logger.h (LOG_* macros), platform_time.h, plus the single-
 * call init/pump/shutdown API. Anything else host-specific that used to
 * live in this file (fd_set / select / termios / stdin polling) is gone
 * — platform_porting_pump() feeds the shell now. */
#include "platform_porting.h"

/* util_parse_u32 / util_parse_i32 for the shell command handlers
 * (same helpers used by motor_drive_master). */
#include "utils.h"

#ifdef USE_LINUX_CAN
#include "hal_can_udp.h"
#include "hal_can_zlg.h"
#endif

#include "tinysh.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>           /* tinysh_char_out still uses putchar    */
                             /* (interactive keystroke echo); every    */
                             /* other output goes through LOG_*.      */
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TAG "drv-sim"


// ========================================================================
// Virtual motor — pure-kinematic plant for protocol testing.
//
// The simulator integrates a single rigid body around one rotational
// axis. Three modes mirror the CiA 402 cyclic-sync families:
//
//   POSITION  (csp/pp)  : closed loop in the simulator. Picks a slew
//                         velocity from `pos_kp * (target - actual)`,
//                         clamps it to ±max_vel, and ramps omega toward
//                         it with bounded |dω/dt| ≤ max_accel.
//   VELOCITY  (csv/pv)  : ramp omega toward target_vel with the same
//                         acceleration limit. No position feedback —
//                         theta integrates freely.
//   TORQUE    (cst/pt)  : apply target_torque directly. Net torque
//                         (target − load) integrates through 1/J to
//                         omega, then to theta. omega still respects
//                         max_vel as a runaway guard.
//
// When NOT enabled (cia402 not in OPERATION_ENABLED) the body coasts
// with mild damping so a bench session that flips the drive off mid-
// motion doesn't freeze with stale velocity.
//
// Everything in SI: rad, rad/s, Nm, kg·m². The cia402 boundary is the
// only place units get translated to / from CANopen increments.
// ========================================================================
#define CTRL_FREQ_HZ         10000
// CiA 402 position increments: radians * (2^POS_INC_BITS) / (2*pi).
// Independent of the physical sensor - this is a transport unit.
#define POS_INC_BITS         14
// Rated torque for the percent-of-rated unit on 0x6071 / 0x6077.
#define RATED_TORQUE_NM      0.5f

typedef struct {
    /* Drive plumbing — set by the cia402 bridges. */
    bool       enabled;
    int32_t    mode;               /* CIA402_MODE_*                */

    /* Rigid-body state. */
    double     theta_rad;
    double     omega_rad_s;
    double     torque_nm_actual;   /* fed back into 0x6077         */

    /* Targets — copied from the cia402 OD once per tick. */
    double     target_pos_rad;
    double     target_vel_rad_s;
    double     target_trq_nm;

    /* Limits / shape — runtime-tunable from the shell. */
    double     max_vel;            /* |omega|       <= max_vel     */
    double     max_accel;          /* |dω/dt|       <= max_accel   */
    double     pos_kp;             /* rad/s per rad of pos error   */

    /* Physics. */
    double     inertia_kgm2;       /* J                            */
    double     load_torque_nm;     /* shell-injectable load        */

    double     dt;                 /* 1 / CTRL_FREQ_HZ             */
} vmotor_t;

static vmotor_t g_vmotor;

static double vm_clamp(double v, double lo, double hi){
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static void vmotor_init(vmotor_t* v){
    memset(v, 0, sizeof(*v));
    v->dt              = 1.0 / (double)CTRL_FREQ_HZ;
    v->inertia_kgm2    = 0.00005;        /* matches the old plant */
    v->load_torque_nm  = 0.0;
    v->max_vel         = 200.0;          /* rad/s   ≈ 1900 rpm    */
    v->max_accel       = 1000.0;         /* rad/s²                */
    v->pos_kp          = 30.0;           /* ≈ 30 Hz position loop */
}

static void vmotor_step(vmotor_t* v){
    if(!v->enabled){
        /* Coast with mild damping when the drive is not in
         * OPERATION_ENABLED. Picks 0.999 per 10 kHz tick = ~10 s
         * time-constant — long enough that you can see the rotor
         * decelerate, short enough not to feel sticky. */
        v->omega_rad_s     *= 0.999;
        v->theta_rad       += v->omega_rad_s * v->dt;
        v->torque_nm_actual = 0.0;
        return;
    }

    const double prev_omega = v->omega_rad_s;
    const double dwmax      = v->max_accel * v->dt;
    double torque_nm        = 0.0;

    switch(v->mode){
    case CIA402_MODE_CYCLIC_SYNC_POSITION:
    case CIA402_MODE_PROFILE_POSITION: {
        const double err          = v->target_pos_rad - v->theta_rad;
        const double omega_target = vm_clamp(v->pos_kp * err,
                                             -v->max_vel, v->max_vel);
        const double dw           = vm_clamp(omega_target - v->omega_rad_s,
                                             -dwmax, dwmax);
        v->omega_rad_s += dw;
        break;
    }
    case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
    case CIA402_MODE_PROFILE_VELOCITY:
    case CIA402_MODE_VELOCITY: {
        const double omega_target = vm_clamp(v->target_vel_rad_s,
                                             -v->max_vel, v->max_vel);
        const double dw           = vm_clamp(omega_target - v->omega_rad_s,
                                             -dwmax, dwmax);
        v->omega_rad_s += dw;
        break;
    }
    case CIA402_MODE_CYCLIC_SYNC_TORQUE:
    case CIA402_MODE_PROFILE_TORQUE: {
        torque_nm = v->target_trq_nm;
        const double accel = (torque_nm - v->load_torque_nm) / v->inertia_kgm2;
        v->omega_rad_s += accel * v->dt;
        v->omega_rad_s  = vm_clamp(v->omega_rad_s, -v->max_vel, v->max_vel);
        break;
    }
    default:
        /* Mode NONE / unknown: coast as if disabled. */
        v->omega_rad_s *= 0.999;
        break;
    }

    v->theta_rad += v->omega_rad_s * v->dt;

    /* In position/velocity modes the "torque" reported back is the
     * inertia-times-acceleration consequence of the slew, plus the
     * load-side disturbance. In torque mode it's the commanded value. */
    if(v->mode != CIA402_MODE_CYCLIC_SYNC_TORQUE &&
       v->mode != CIA402_MODE_PROFILE_TORQUE){
        const double accel = (v->omega_rad_s - prev_omega) / v->dt;
        torque_nm = v->inertia_kgm2 * accel + v->load_torque_nm;
    }
    v->torque_nm_actual = torque_nm;
}


// ========================================================================
// Unit conversions:  motor (SI) <-> CiA 402 increments
// ========================================================================
#define POS_COUNTS_PER_REV   (1u << POS_INC_BITS)
#define INC_PER_RAD          ((double)POS_COUNTS_PER_REV / (2.0 * M_PI))

static inline int32_t rad_to_inc(float rad){
    return (int32_t)((double)rad * INC_PER_RAD);
}
static inline float inc_to_rad(int32_t inc){
    return (float)((double)inc / INC_PER_RAD);
}

// Torque: CiA 402 units are 0.1% of rated (1000 = 100% rated).
static inline int16_t nm_to_milli_rated(float nm){
    int32_t v = (int32_t)((nm / RATED_TORQUE_NM) * 1000.0f);
    if(v > 32767){ v = 32767; }
    if(v < -32768){ v = -32768; }
    return (int16_t)v;
}
static inline float milli_rated_to_nm(int16_t v){
    return ((float)v / 1000.0f) * RATED_TORQUE_NM;
}


// ========================================================================
// Virtual CAN bus (identical pattern to example/can_stack_shell.c)
// ========================================================================
#define VBUS_ENDPOINT_MAX   4

typedef struct vbus vbus_t;

typedef struct {
    hal_can_t       base;
    vbus_t*         bus;
    const char*     name;
    hal_can_rx_fn_t rx_cb;
    void*           rx_arg;
} vbus_endpoint_t;

struct vbus {
    vbus_endpoint_t eps[VBUS_ENDPOINT_MAX];
    int             n_eps;
    uint32_t        tx_total;
};

static int32_t vb_write(hal_can_t* h, const can_msg_t* m){
    vbus_endpoint_t* src = (vbus_endpoint_t*)h;
    src->bus->tx_total++;
    for(int i = 0; i < src->bus->n_eps; i++){
        vbus_endpoint_t* ep = &src->bus->eps[i];
        if(!ep->rx_cb){ continue; }
        ep->rx_cb(m, ep->rx_arg);
    }
    return 0;
}
static int32_t vb_set_cb(hal_can_t* h, hal_can_rx_fn_t rx, hal_can_tx_fn_t tx, void* arg){
    (void)tx;
    vbus_endpoint_t* ep = (vbus_endpoint_t*)h;
    ep->rx_cb  = rx;
    ep->rx_arg = arg;
    return 0;
}
static int32_t vb_init(hal_can_t* h, uint32_t a, uint32_t b){ (void)h; (void)a; (void)b; return 0; }
static int32_t vb_nop (hal_can_t* h){ (void)h; return 0; }
static int32_t vb_filt(hal_can_t* h, uint8_t i, uint32_t a, uint32_t b){
    (void)h; (void)i; (void)a; (void)b; return 0;
}
static int32_t vb_stat(hal_can_t* h, uint32_t* out){ (void)h; if(out){ *out = CAN_BUS_OK; } return 0; }

static const hal_can_proc_t s_vb_proc = {
    .init = vb_init, .free = vb_nop, .start = vb_nop, .stop = vb_nop,
    .write = vb_write, .set_cb = vb_set_cb, .set_filter = vb_filt,
    .get_bus_status = vb_stat, .bus_off_recovery = vb_nop,
};

static void vbus_init(vbus_t* b){ memset(b, 0, sizeof(*b)); }
static hal_can_t* vbus_attach(vbus_t* b, const char* nm){
    if(b->n_eps >= VBUS_ENDPOINT_MAX){ return NULL; }
    vbus_endpoint_t* ep = &b->eps[b->n_eps++];
    memset(ep, 0, sizeof(*ep));
    ep->base.proc = &s_vb_proc;
    ep->bus = b;
    ep->name = nm;
    return &ep->base;
}


// ========================================================================
// Application context
// ========================================================================
#define DRIVE_NODE_ID       5
#define MASTER_NODE_ID      7

// Manufacturer-specific OD entry exposed so a master can reprogram the
// node id at runtime. Writing a byte to 0x2000.01 retargets the USDO
// server AND the default PDO COB-IDs on this node.
#define OD_NODE_ID_INDEX    (0x2000u)
#define OD_NODE_ID_SUB      (0x01u)

typedef struct {
    vbus_t               bus;

    // drive side: pure-virtual motor + cia402 + USDO server + PDO
    cia402_t*            drive;
    can_fd_pdo_t*        pdo;
    co_fd_usdo_server_t* srv;
    co_od_t              drive_od;

    // master side (NULL in UDP mode; external master takes over)
    co_fd_usdo_client_t* cli;

    // Backing hal_can_t handles. In vbus mode these point into g_app.bus.
    // In UDP mode they're two endpoints on the same multicast group so the
    // kernel cross-delivers frames between the PDO layer and USDO server.
    hal_can_t*           hcan_srv;
    hal_can_t*           hcan_pdo;

    // CLI: pick one transport (default ZLG).
    //   --zlg (default) -> real CAN-FD over a ZLG USBCANFD-x00U adapter
    //   --udp           -> UDP-multicast loopback "CAN bus"
    //   --vbus          -> in-process virtual bus (drive + master in one binary)
    bool                 use_udp;
    bool                 use_zlg;
    bool                 use_vbus;
    const char*          udp_group;   // NULL = hal_can_udp default
    uint16_t             udp_port;    // 0    = hal_can_udp default
    hal_can_zlg_cfg_t    zlg_cfg;     // populated by --zlg-* flags

    // Runtime-settable node ID (seeded to DRIVE_NODE_ID). Mirrored into
    // the 0x2000.01 OD sub-object so a master can change it over SDO.
    uint8_t              node_id;

    // Storage for the OD entry at 0x2000.01 + its owning co_obj_t. Must
    // live as long as drive_od references them.
    co_sub_obj_t         node_id_subs[2];
    co_obj_t             node_id_obj;
    co_obj_t*            merged_od_list;   // malloc'd: cia402 objs + extras

    // Optional standard drive entries (0x6072 / 0x6076 / 0x6078 /
    // 0x607C / 0x607D / 0x6080) — see cia402_drive_od.h.
    cia402_drive_od_t    extra_od;

    /* Event-driven RPDO flag: set by on_rpdo_received() inside
     * can_fd_pdo_process() each time an RPDO matches. The main loop
     * consumes the edge once per iteration — adopts targets into the
     * vmotor (only when state == OPERATION_ENABLED) and triggers a
     * paired TPDO so master/slave traffic stays 1:1. */
    bool                 rpdo_pending;

    /* Timestamp (ms) of the most recent TPDO emission. Used to gate
     * the 10 Hz heartbeat trigger that keeps the master's cached
     * actuals fresh even when no RPDO is flowing (pre-bringup). */
    uint32_t             last_tpdo_ms;

    // bookkeeping
    int                  quit;
} app_ctx_t;

static app_ctx_t g_app;


void tinysh_char_out(unsigned char c){
    putchar((int)c);
    fflush(stdout);
}


// ========================================================================
// Node-ID retarget (write hook on OD 0x2000.01)
// ========================================================================

/* RPDO receive callback. Fires inside can_fd_pdo_process() before the
 * PDO mapping scatters the payload bytes into the OD — so we only trip
 * the edge here and let the main loop do the work on the next pass,
 * once the OD reflects the new targets. */
static void on_rpdo_received(uint32_t _cob_id, const uint8_t* _data,
                             uint8_t _len, void* _arg)
{
    (void)_cob_id; (void)_data; (void)_len;
    app_ctx_t* x = (app_ctx_t*)_arg;
    x->rpdo_pending = true;
}

static void reconfigure_default_pdos(app_ctx_t* x, uint8_t new_node_id){
    // Default CANopen predefined connection set:
    //   RPDO1 cob = 0x200 + id    TPDO1 cob = 0x180 + id
    // Clear the old slot first so any cached filter/mask is dropped.
    can_fd_rpdo_remove(x->pdo, 0);
    can_fd_tpdo_remove(x->pdo, 0);

    // RPDO1: Controlword + Target{Position, Velocity, Torque}  = 12 bytes.
    // .cb fires per matched frame so the main loop can pair each RPDO
    // with an apply-targets-then-TPDO response (event-driven cyclic sync).
    co_rpdo_config_t rpdo_cfg = {
        .cob_id     = 0x200u + new_node_id,
        .trans_type = CO_PDO_TX_ASYNC_MFR,
        .data_len   = 12,
        .cb         = on_rpdo_received,
        .arg        = x,
    };
    can_fd_rpdo_configure(x->pdo, 0, &rpdo_cfg);
    co_pdo_map_entry_t rpdo_map[] = {
        { .od_index = 0x6040, .od_sub_index = 0, .bit_length = 16 },  // Controlword
        { .od_index = 0x607A, .od_sub_index = 0, .bit_length = 32 },  // Target Position
        { .od_index = 0x60FF, .od_sub_index = 0, .bit_length = 32 },  // Target Velocity
        { .od_index = 0x6071, .od_sub_index = 0, .bit_length = 16 },  // Target Torque
    };
    can_fd_rpdo_set_mapping(x->pdo, 0, rpdo_map, 4);

    // TPDO1: Statusword + {Position, Velocity, Torque}Actual + ErrorCode = 14 bytes.
    // Auto-emit @ 2 kHz driven by the slave main-loop tick (inhibit_time=0
    // so app-driven triggers aren't suppressed).
    co_tpdo_config_t tpdo_cfg = {
        .cob_id             = 0x180u + new_node_id,
        .trans_type         = CO_PDO_TX_ASYNC_MFR,
        .data_len           = 14,
        .inhibit_time_100us = 0,
        .event_timer_ms     = 0,
    };
    can_fd_tpdo_configure(x->pdo, 0, &tpdo_cfg);
    co_pdo_map_entry_t tpdo_map[] = {
        { .od_index = 0x6041, .od_sub_index = 0, .bit_length = 16 },  // Statusword
        { .od_index = 0x6064, .od_sub_index = 0, .bit_length = 32 },  // Position Actual
        { .od_index = 0x606C, .od_sub_index = 0, .bit_length = 32 },  // Velocity Actual
        { .od_index = 0x6077, .od_sub_index = 0, .bit_length = 16 },  // Torque Actual
        { .od_index = 0x603F, .od_sub_index = 0, .bit_length = 16 },  // Error Code
    };
    can_fd_tpdo_set_mapping(x->pdo, 0, tpdo_map, 5);
}

// Fires after an SDO download has written the new byte into g_app.node_id.
// Re-targets the USDO server's accepted COB-ID AND the default PDO slot
// so everything uses the new id without a reboot.
static void od_node_id_written(uint16_t idx, uint8_t sub, void* arg){
    (void)idx; (void)sub;
    app_ctx_t* x = (app_ctx_t*)arg;
    uint8_t new_id = x->node_id;
    if(new_id == 0u || new_id > 127u){
        LOG_ERR(TAG, "node-id rejected %u (must be 1..127)", (unsigned)new_id);
        return;
    }

    LOG_INFO(TAG, "node-id retarget -> %u", (unsigned)new_id);
    co_fd_usdo_server_set_node_id(x->srv, new_id);
    reconfigure_default_pdos(x, new_id);
}

// Build drive_od as (cia402 objects) ++ (cia402_drive_od extras) ++
// (one 0x2000 node-id object).
static int32_t build_merged_od(app_ctx_t* x){
    uint16_t cia_count = 0;
    co_obj_t* cia_objs = cia402_get_od_objects(x->drive, &cia_count);
    if(!cia_objs){ return -1; }

    cia402_drive_od_init(&x->extra_od);
    co_obj_t* extra_objs  = cia402_drive_od_objs(&x->extra_od);
    uint16_t  extra_count = (uint16_t)CIA402_DRIVE_OD_COUNT;

    uint16_t total = cia_count + extra_count + 1u;
    x->merged_od_list = (co_obj_t*)calloc(total, sizeof(co_obj_t));
    if(!x->merged_od_list){ return -1; }
    memcpy(x->merged_od_list,                 cia_objs,
           cia_count   * sizeof(co_obj_t));
    memcpy(x->merged_od_list + cia_count,     extra_objs,
           extra_count * sizeof(co_obj_t));

    // 0x2000.00: "Number of entries" (U8, const 1).
    static uint8_t nsubs = 1u;
    x->node_id_subs[0].data           = &nsubs;
    x->node_id_subs[0].len            = 1;
    x->node_id_subs[0].data_type      = CO_DT_UINT8;
    x->node_id_subs[0].access         = CO_ACCESS_CONST;
    x->node_id_subs[0].data_access_fn = NULL;

    // 0x2000.01: Node ID byte — RW, fires od_node_id_written on download.
    x->node_id_subs[1].data           = &x->node_id;
    x->node_id_subs[1].len            = 1;
    x->node_id_subs[1].data_type      = CO_DT_UINT8;
    x->node_id_subs[1].access         = CO_ACCESS_RW;
    x->node_id_subs[1].data_access_fn = od_node_id_written;
    x->node_id_subs[1].arg            = x;

    x->node_id_obj.index      = OD_NODE_ID_INDEX;
    x->node_id_obj.sub_number = 2;
    x->node_id_obj.subs       = x->node_id_subs;

    x->merged_od_list[cia_count + extra_count] = x->node_id_obj;

    x->drive_od.number = total;
    x->drive_od.list   = x->merged_od_list;
    return 0;
}


// ========================================================================
// Bridge: cia402 <-> virtual motor
// ========================================================================
static void on_cia402_state(cia402_state_t old, cia402_state_t neu, void* arg){
    (void)arg;
    /* OPERATION_ENABLED is the only "live" state. Anything else parks
     * the simulator (it coasts with mild damping in vmotor_step). */
    g_vmotor.enabled = (neu == CIA402_STATE_OPERATION_ENABLED);
    LOG_INFO(TAG, "cia402 state 0x%04X -> 0x%04X  vmotor=%s",
             (unsigned)old, (unsigned)neu,
             g_vmotor.enabled ? "ENABLED" : "coast");
}

static void on_cia402_mode(cia402_mode_t old, cia402_mode_t neu, void* arg){
    (void)arg;
    g_vmotor.mode = (int32_t)neu;
    LOG_INFO(TAG, "cia402 mode %d -> %d", (int)old, (int)neu);
}

/* Notional torque constant for deriving 0x6078 current_actual from
 * vmotor torque. The simulator has no real current loop — this just
 * gives the master a plausibly-shaped value to read back. */
#define VMOTOR_KT_NM_PER_A      (0.3f)

/* RPM <-> rad/s conversion factor. CiA 402 §7 specifies 0x6080 in rpm. */
#define RPM_TO_RAD_PER_S        (2.0f * (float)M_PI / 60.0f)

// Once per tick while the drive is enabled: forward targets written into
// the cia402 OD to the virtual motor (in SI units), applying the
// software limits from the cia402_drive_od_t extras (0x6072 / 0x607D /
// 0x6080).
static void apply_cia402_targets(app_ctx_t* a){
    if(cia402_get_state(a->drive) != CIA402_STATE_OPERATION_ENABLED){
        return;
    }
    int32_t tp = 0, tv = 0;
    int16_t tt = 0;
    cia402_get_target_position(a->drive, &tp);
    cia402_get_target_velocity(a->drive, &tv);
    cia402_get_target_torque  (a->drive, &tt);

    /* 0x607D software position limit. Clamp the target before the
     * vmotor sees it, so the simulator never tries to slew past the
     * configured envelope. The home-offset (0x607C) is applied at
     * publish-time only, so master-side targets refer to absolute
     * post-homed coordinates. */
    if(tp < a->extra_od.pos_limit_min){ tp = a->extra_od.pos_limit_min; }
    if(tp > a->extra_od.pos_limit_max){ tp = a->extra_od.pos_limit_max; }

    /* 0x6080 max motor speed (rpm). Convert to rad/s for the magnitude
     * clamp. */
    const float max_v_rad_s =
        (float)a->extra_od.max_motor_speed * RPM_TO_RAD_PER_S;
    float tv_rad_s = inc_to_rad(tv);
    if(tv_rad_s >  max_v_rad_s){ tv_rad_s =  max_v_rad_s; }
    if(tv_rad_s < -max_v_rad_s){ tv_rad_s = -max_v_rad_s; }

    /* 0x6072 max torque (per-mille of rated). Cap |target_trq|. */
    int16_t mt = (int16_t)((a->extra_od.max_torque > 32767u)
                           ? 32767u : a->extra_od.max_torque);
    if(tt >  mt){ tt =  mt; }
    if(tt < -mt){ tt = -mt; }

    g_vmotor.target_pos_rad   = (double)inc_to_rad(tp);
    g_vmotor.target_vel_rad_s = (double)tv_rad_s;
    g_vmotor.target_trq_nm    = (double)milli_rated_to_nm(tt);
}

// Once per tick: snapshot virtual-motor state into cia402 actuals so
// statusword / TxPDOs reflect simulator reality. Applies the home
// offset (0x607C) and derives a stand-in current value for 0x6078.
static void publish_motor_actuals(app_ctx_t* a){
    /* Home offset shifts the reported position toward the master.
     * Increments-domain subtract: pos_actual = theta_inc - home_offset. */
    int32_t pos_inc = rad_to_inc((float)g_vmotor.theta_rad)
                    - a->extra_od.home_offset;
    cia402_set_position_actual(a->drive, pos_inc);
    cia402_set_velocity_actual(a->drive, rad_to_inc((float)g_vmotor.omega_rad_s));
    cia402_set_torque_actual  (a->drive, nm_to_milli_rated((float)g_vmotor.torque_nm_actual));

    /* 0x6078 current actual, in mA. The simulator has no current loop;
     * approximate from torque using a notional Kt so the master sees a
     * physically-shaped reading. */
    float i_amps = (float)g_vmotor.torque_nm_actual / VMOTOR_KT_NM_PER_A;
    int32_t i_mA = (int32_t)(i_amps * 1000.0f);
    if(i_mA >  32767){ i_mA =  32767; }
    if(i_mA < -32768){ i_mA = -32768; }
    a->extra_od.current_actual_mA = (int16_t)i_mA;
}


// ========================================================================
// Control tick (10 kHz)
// ========================================================================
static void tick_motor_once(app_ctx_t* a){
    (void)a;
    vmotor_step(&g_vmotor);
}


// ========================================================================
// Shell commands
// ========================================================================
// parse_u32 / parse_i32 moved to src/common/utils.h → util_parse_u32 /
// util_parse_i32. Call sites below updated in lockstep.

static const char* state_name(cia402_state_t s){
    switch(s){
    case CIA402_STATE_NOT_READY_TO_SWITCH_ON: return "NOT_READY";
    case CIA402_STATE_SWITCH_ON_DISABLED:     return "SWITCH_ON_DISABLED";
    case CIA402_STATE_READY_TO_SWITCH_ON:     return "READY_TO_SWITCH_ON";
    case CIA402_STATE_SWITCHED_ON:            return "SWITCHED_ON";
    case CIA402_STATE_OPERATION_ENABLED:      return "OPERATION_ENABLED";
    case CIA402_STATE_QUICK_STOP_ACTIVE:      return "QUICK_STOP_ACTIVE";
    case CIA402_STATE_FAULT_REACTION_ACTIVE:  return "FAULT_REACTION_ACTIVE";
    case CIA402_STATE_FAULT:                  return "FAULT";
    default:                                  return "?";
    }
}


static void cmd_status(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    LOG_INFO(TAG, "cia402 state : %s  (sw=0x%04X)",
             state_name(cia402_get_state(x->drive)),
             cia402_get_statusword(x->drive));
    LOG_INFO(TAG, "cia402 mode  : %d", (int)cia402_get_mode(x->drive));
    LOG_INFO(TAG, "vmotor       : %s   mode=%d",
             g_vmotor.enabled ? "ENABLED" : "coast",
             (int)g_vmotor.mode);
    LOG_INFO(TAG, "error code   : 0x%04X", cia402_get_error_code(x->drive));
}

static void cmd_report(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    int32_t tp = 0, tv = 0;
    int16_t tt = 0;
    cia402_get_target_position(x->drive, &tp);
    cia402_get_target_velocity(x->drive, &tv);
    cia402_get_target_torque  (x->drive, &tt);
    LOG_INFO(TAG, "vmotor pos=%+7.3f rad  vel=%+8.2f rad/s  T=%+6.3f Nm",
             g_vmotor.theta_rad, g_vmotor.omega_rad_s,
             g_vmotor.torque_nm_actual);
    LOG_INFO(TAG, "cia402 actual 0x6064=%d  0x606C=%d  0x6077=%d",
             rad_to_inc((float)g_vmotor.theta_rad),
             rad_to_inc((float)g_vmotor.omega_rad_s),
             nm_to_milli_rated((float)g_vmotor.torque_nm_actual));
    LOG_INFO(TAG, "cia402 target 0x607A=%d  0x60FF=%d  0x6071=%d",
             tp, tv, (int)tt);
    LOG_INFO(TAG, "vm limits  max_vel=%.1f rad/s  max_accel=%.0f rad/s^2  pos_kp=%.1f",
             g_vmotor.max_vel, g_vmotor.max_accel, g_vmotor.pos_kp);
    LOG_INFO(TAG, "vm physics J=%.2e kgm^2  load=%+6.3f Nm",
             g_vmotor.inertia_kgm2, g_vmotor.load_torque_nm);
    LOG_INFO(TAG, "OD-ext  0x6072 max_torque=%u (per-mille rated)  0x6076 rated=%u uNm",
             (unsigned)x->extra_od.max_torque,
             (unsigned)x->extra_od.motor_rated_torque_uNm);
    LOG_INFO(TAG, "OD-ext  0x6078 cur_actual=%d mA  0x607C home_offset=%d  0x6080 max_speed=%u rpm",
             (int)x->extra_od.current_actual_mA,
             (int)x->extra_od.home_offset,
             (unsigned)x->extra_od.max_motor_speed);
    LOG_INFO(TAG, "OD-ext  0x607D pos_limit min=%d max=%d",
             (int)x->extra_od.pos_limit_min,
             (int)x->extra_od.pos_limit_max);
}

// --- Local controlword commands (no CAN in between) ---------------------
static void cmd_shutdown(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, CIA402_CW_SHUTDOWN);
}
static void cmd_switchon(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, CIA402_CW_SWITCHON);
}
static void cmd_enable(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, CIA402_CW_ENABLE_OPERATION);
}
static void cmd_disable(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, CIA402_CW_DISABLE_OPERATION);
}
static void cmd_quickstop(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, CIA402_CW_QUICK_STOP);
}
static void cmd_reset(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    cia402_set_controlword(x->drive, 0);
    cia402_set_controlword(x->drive, CIA402_CW_FAULT_RESET);
}
static void cmd_mode(int argc, char** argv){
    app_ctx_t* x = tinysh_get_arg();
    if(argc < 2){
        LOG_INFO(TAG, "usage: mode <csp|csv|cst|pp|pv|pt|velocity|none|<n>>");
        return;
    }
    int16_t m = CIA402_MODE_NONE;
    if     (!strcmp(argv[1], "csp"))      m = CIA402_MODE_CYCLIC_SYNC_POSITION;
    else if(!strcmp(argv[1], "csv"))      m = CIA402_MODE_CYCLIC_SYNC_VELOCITY;
    else if(!strcmp(argv[1], "cst"))      m = CIA402_MODE_CYCLIC_SYNC_TORQUE;
    else if(!strcmp(argv[1], "pp"))       m = CIA402_MODE_PROFILE_POSITION;
    else if(!strcmp(argv[1], "pv"))       m = CIA402_MODE_PROFILE_VELOCITY;
    else if(!strcmp(argv[1], "pt"))       m = CIA402_MODE_PROFILE_TORQUE;
    else if(!strcmp(argv[1], "velocity")) m = CIA402_MODE_VELOCITY;
    else if(!strcmp(argv[1], "none"))     m = CIA402_MODE_NONE;
    else                                  m = (int16_t)util_parse_i32(argv[1]);
    cia402_set_mode_of_operation(x->drive, m);
}

// Target commands operate on the cia402 OD directly, which is the same
// storage an RPDO scatter or SDO download would modify.
static void cmd_target(int argc, char** argv){
    app_ctx_t* x = tinysh_get_arg();
    if(argc < 3){
        LOG_INFO(TAG, "usage: target <pos|vel|torque> <value>");
        return;
    }
    int32_t v = util_parse_i32(argv[2]);
    if(!strcmp(argv[1], "pos")){
        co_od_write(&x->drive_od, 0x607A, 0, &v, 4);
        LOG_INFO(TAG, "target pos <- %d inc  (%.3f rad)", v, inc_to_rad(v));
    }
    else if(!strcmp(argv[1], "vel")){
        co_od_write(&x->drive_od, 0x60FF, 0, &v, 4);
        LOG_INFO(TAG, "target vel <- %d inc/s  (%.2f rad/s)", v, inc_to_rad(v));
    }
    else if(!strcmp(argv[1], "torque")){
        int16_t t = (int16_t)v;
        co_od_write(&x->drive_od, 0x6071, 0, &t, 2);
        LOG_INFO(TAG, "target torque <- %d (0.1%%-rated)  (%.3f Nm)",
                 (int)t, milli_rated_to_nm(t));
    }
    else {
        LOG_WARN(TAG, "bad field");
    }
}

static void cmd_load(int argc, char** argv){
    if(argc < 2){ LOG_INFO(TAG, "usage: load <Nm>"); return; }
    g_vmotor.load_torque_nm = atof(argv[1]);
    LOG_INFO(TAG, "vmotor load = %.3f Nm", g_vmotor.load_torque_nm);
}

/* ---- Virtual-motor knobs ------------------------------------------- */
static void cmd_vm_max_vel(int argc, char** argv){
    if(argc < 2){
        LOG_INFO(TAG, "usage: vm_max_vel <rad/s>   (current %.1f)",
                 g_vmotor.max_vel);
        return;
    }
    double v = atof(argv[1]);
    if(v < 0.0){ v = -v; }
    g_vmotor.max_vel = v;
    LOG_INFO(TAG, "vmotor max_vel = %.2f rad/s", g_vmotor.max_vel);
}

static void cmd_vm_max_accel(int argc, char** argv){
    if(argc < 2){
        LOG_INFO(TAG, "usage: vm_max_accel <rad/s^2>  (current %.1f)",
                 g_vmotor.max_accel);
        return;
    }
    double a = atof(argv[1]);
    if(a < 0.0){ a = -a; }
    g_vmotor.max_accel = a;
    LOG_INFO(TAG, "vmotor max_accel = %.2f rad/s^2", g_vmotor.max_accel);
}

static void cmd_vm_pos_kp(int argc, char** argv){
    if(argc < 2){
        LOG_INFO(TAG, "usage: vm_pos_kp <rad/s per rad>  (current %.2f)",
                 g_vmotor.pos_kp);
        return;
    }
    g_vmotor.pos_kp = atof(argv[1]);
    LOG_INFO(TAG, "vmotor pos_kp = %.3f rad/s per rad", g_vmotor.pos_kp);
}

static void cmd_vm_inertia(int argc, char** argv){
    if(argc < 2){
        LOG_INFO(TAG, "usage: vm_inertia <kg*m^2>  (current %.3e)",
                 g_vmotor.inertia_kgm2);
        return;
    }
    double j = atof(argv[1]);
    if(j > 1e-12){
        g_vmotor.inertia_kgm2 = j;
        LOG_INFO(TAG, "vmotor inertia = %.3e kg*m^2", g_vmotor.inertia_kgm2);
    } else {
        LOG_WARN(TAG, "inertia must be > 0");
    }
}

static void cmd_vm_reset(int argc, char** argv){
    (void)argc; (void)argv;
    /* Snap rotor back to the origin without touching the cia402 layer.
     * Useful between test cases — clean known starting point for the
     * master to walk through bring-up again. */
    g_vmotor.theta_rad        = 0.0;
    g_vmotor.omega_rad_s      = 0.0;
    g_vmotor.torque_nm_actual = 0.0;
    g_vmotor.target_pos_rad   = 0.0;
    g_vmotor.target_vel_rad_s = 0.0;
    g_vmotor.target_trq_nm    = 0.0;
    LOG_INFO(TAG, "vmotor reset (theta=omega=0, targets cleared)");
}

static void cmd_fault(int argc, char** argv){
    app_ctx_t* x = tinysh_get_arg();
    uint16_t code = (argc >= 2) ? (uint16_t)util_parse_u32(argv[1])
                                : (uint16_t)CIA402_ERR_OVER_CURRENT;
    cia402_raise_fault(x->drive, code);
}


// --- SDO commands via the master-node USDO client (remote path) ---------
static volatile int g_sdo_done;
static uint8_t  g_sdo_rc;
static uint8_t  g_sdo_buf[64];

static void on_sdo_done(uint8_t rc, uint16_t idx, uint8_t sub, void* a){
    (void)a;
    g_sdo_rc   = rc;
    g_sdo_done = 1;
    LOG_INFO(TAG, "SDO 0x%04X.%u done rc=%u", idx, sub, rc);
}

static void cmd_sdo_get(int argc, char** argv){
    app_ctx_t* x = tinysh_get_arg();
    if(!x->cli){
        LOG_WARN(TAG, "sdo_get disabled: no in-process master (UDP mode)");
        return;
    }
    if(argc < 3){ LOG_INFO(TAG, "usage: sdo_get <idx> <sub>"); return; }
    uint16_t idx = (uint16_t)util_parse_u32(argv[1]);
    uint8_t  sub = (uint8_t) util_parse_u32(argv[2]);
    memset(g_sdo_buf, 0, sizeof(g_sdo_buf));
    co_sub_obj_t obj = {
        .data = g_sdo_buf, .len = sizeof(g_sdo_buf),
        .data_type = CO_DT_OCTET_STR, .access = CO_ACCESS_RW,
    };
    g_sdo_done = 0;
    int r = co_fd_usdo_client_upload(x->cli, DRIVE_NODE_ID, idx, sub,
                                     &obj, 1000, on_sdo_done, NULL);
    LOG_INFO(TAG, "sdo_get -> %d", r);
}

static void cmd_sdo_put(int argc, char** argv){
    app_ctx_t* x = tinysh_get_arg();
    if(!x->cli){
        LOG_WARN(TAG, "sdo_put disabled: no in-process master (UDP mode)");
        return;
    }
    if(argc < 4){ LOG_INFO(TAG, "usage: sdo_put <idx> <sub> <u32>"); return; }
    uint16_t idx = (uint16_t)util_parse_u32(argv[1]);
    uint8_t  sub = (uint8_t) util_parse_u32(argv[2]);
    uint32_t v   = util_parse_u32(argv[3]);
    // Fixed-size CANopen entries expect an exact length; probe the drive
    // OD locally for the target length (a real master would already know
    // the type from EDS/object-dictionary metadata).
    co_sub_obj_t* s = co_od_find_obj(&x->drive_od, idx, sub);
    uint32_t len = (s && s->len > 0 && s->len <= 4) ? s->len : 4;
    memcpy(g_sdo_buf, &v, len);
    co_sub_obj_t obj = {
        .data = g_sdo_buf, .len = len,
        .data_type = CO_DT_OCTET_STR, .access = CO_ACCESS_RW,
    };
    g_sdo_done = 0;
    int r = co_fd_usdo_client_download(x->cli, DRIVE_NODE_ID, idx, sub,
                                       &obj, 1000, on_sdo_done, NULL);
    LOG_INFO(TAG, "sdo_put -> %d (%u bytes)", r, (unsigned)len);
}


static void cmd_quit(int argc, char** argv){
    (void)argc; (void)argv;
    app_ctx_t* x = tinysh_get_arg();
    x->quit = 1;
}


#define CMD(N, H, F) static tinysh_cmd_t cmdtab_##F = { 0, (N), (H), 0, (F), &g_app, 0, 0 }

CMD("status",       "cia402 + vmotor state",                    cmd_status);
CMD("report",       "actuals + targets + vm tunables",          cmd_report);
CMD("shutdown",     "cia402 shutdown command",                  cmd_shutdown);
CMD("switchon",     "cia402 switch-on command",                 cmd_switchon);
CMD("enable",       "cia402 enable-operation",                  cmd_enable);
CMD("disable",      "cia402 disable-operation",                 cmd_disable);
CMD("quickstop",    "cia402 quick-stop",                        cmd_quickstop);
CMD("reset",        "cia402 fault reset",                       cmd_reset);
CMD("mode",         "set mode of operation",                    cmd_mode);
CMD("target",       "write target pos/vel/torque",              cmd_target);
CMD("load",         "inject vmotor load (Nm)",                  cmd_load);
CMD("vm_max_vel",   "vmotor velocity limit (rad/s)",            cmd_vm_max_vel);
CMD("vm_max_accel", "vmotor accel limit (rad/s^2)",             cmd_vm_max_accel);
CMD("vm_pos_kp",    "vmotor position-mode P gain",              cmd_vm_pos_kp);
CMD("vm_inertia",   "vmotor inertia (kg*m^2)",                  cmd_vm_inertia);
CMD("vm_reset",     "snap vmotor rotor back to origin",         cmd_vm_reset);
CMD("fault",        "inject cia402 fault (code)",               cmd_fault);
CMD("sdo_get",      "SDO upload (master -> drive)",             cmd_sdo_get);
CMD("sdo_put",      "SDO download (master -> drive)",           cmd_sdo_put);
CMD("quit",         "exit",                                     cmd_quit);


// ========================================================================
// Main loop
//
// stdin polling + tty_raw handling used to live here. They're gone —
// platform_porting_pump() drains the shell UART (stdin on host, the
// board's serial peripheral on an MCU) and feeds tinysh_char_in. One
// call per main-loop iteration.
// ========================================================================

int main(int argc, char** argv){
    // Seed the node id with the compile-time default; --node overrides.
    uint8_t cli_node_id = DRIVE_NODE_ID;

    // ZLG is the default transport when no flag is given. Overridable
    // via --zlg-dev-type / --zlg-dev-idx / --zlg-chan / --zlg-abaud /
    // --zlg-dbaud, or replaced entirely with --udp / --vbus.
    g_app.use_zlg                   = true;
    g_app.zlg_cfg.device_type       = HAL_CAN_ZLG_DEFAULT_DEVICE_TYPE;
    g_app.zlg_cfg.device_index      = HAL_CAN_ZLG_DEFAULT_DEVICE_INDEX;
    g_app.zlg_cfg.channel_index     = HAL_CAN_ZLG_DEFAULT_CHANNEL;
    g_app.zlg_cfg.arb_baud_bps      = HAL_CAN_ZLG_DEFAULT_ARB_BPS;
    g_app.zlg_cfg.data_baud_bps     = HAL_CAN_ZLG_DEFAULT_DATA_BPS;
    g_app.zlg_cfg.enable_terminator = 1u;
    g_app.zlg_cfg.iso_canfd         = 1u;

    // --- CLI flags -----------------------------------------------
    for(int i = 1; i < argc; i++){
        if(!strcmp(argv[i], "--udp")){
            g_app.use_udp  = true;
            g_app.use_zlg  = false;
            g_app.use_vbus = false;
            // Optional "group:port" argument consumed if present.
            if(i + 1 < argc && argv[i+1][0] != '-'){
                char* buf = strdup(argv[++i]);
                char* colon = strchr(buf, ':');
                if(colon){
                    *colon = '\0';
                    g_app.udp_group = strdup(buf);
                    g_app.udp_port  = (uint16_t)strtoul(colon + 1, NULL, 0);
                }
                else {
                    g_app.udp_group = strdup(buf);
                }
                free(buf);
            }
        }
        else if(!strcmp(argv[i], "--zlg")){
            g_app.use_zlg  = true;
            g_app.use_udp  = false;
            g_app.use_vbus = false;
        }
        else if(!strcmp(argv[i], "--vbus")){
            g_app.use_vbus = true;
            g_app.use_zlg  = false;
            g_app.use_udp  = false;
        }
        else if(!strcmp(argv[i], "--zlg-dev-type") && i + 1 < argc){
            g_app.zlg_cfg.device_type = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if(!strcmp(argv[i], "--zlg-dev-idx") && i + 1 < argc){
            g_app.zlg_cfg.device_index = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if(!strcmp(argv[i], "--zlg-chan") && i + 1 < argc){
            g_app.zlg_cfg.channel_index = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if(!strcmp(argv[i], "--zlg-abaud") && i + 1 < argc){
            g_app.zlg_cfg.arb_baud_bps = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if(!strcmp(argv[i], "--zlg-dbaud") && i + 1 < argc){
            g_app.zlg_cfg.data_baud_bps = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if((!strcmp(argv[i], "--node") || !strcmp(argv[i], "-n")) && i + 1 < argc){
            unsigned long n = strtoul(argv[++i], NULL, 0);
            if(n >= 1 && n <= 127){
                cli_node_id = (uint8_t)n;
            } else {
                LOG_ERR(TAG, "--node must be 1..127 (got %lu)", n);
                return 2;
            }
        }
        else if(!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")){
            LOG_INFO(TAG, "Usage: %s [--zlg [zlg-opts] | --udp [group:port] | --vbus] [--node <id>]",
                     argv[0]);
            LOG_INFO(TAG, "  --zlg (default)  bind PDO/USDO-server to a ZLG USBCANFD-x00U");
            LOG_INFO(TAG, "                   adapter (defaults: type %u, dev %u, ch %u,",
                     (unsigned)HAL_CAN_ZLG_DEFAULT_DEVICE_TYPE,
                     (unsigned)HAL_CAN_ZLG_DEFAULT_DEVICE_INDEX,
                     (unsigned)HAL_CAN_ZLG_DEFAULT_CHANNEL);
            LOG_INFO(TAG, "                   %u abps / %u dbps).",
                     (unsigned)HAL_CAN_ZLG_DEFAULT_ARB_BPS,
                     (unsigned)HAL_CAN_ZLG_DEFAULT_DATA_BPS);
            LOG_INFO(TAG, "  --udp            bind to UDP-multicast \"CAN bus\" on localhost");
            LOG_INFO(TAG, "                   instead of the ZLG adapter (offline testing).");
            LOG_INFO(TAG, "  --vbus           in-process virtual bus with a shell-owned");
            LOG_INFO(TAG, "                   master client (drive + master in one binary).");
            LOG_INFO(TAG, "  --zlg-dev-type   ZLG device type (default %u = USBCANFD_200U)",
                     (unsigned)HAL_CAN_ZLG_DEFAULT_DEVICE_TYPE);
            LOG_INFO(TAG, "  --zlg-dev-idx    ZLG device index (default 0)");
            LOG_INFO(TAG, "  --zlg-chan       ZLG channel index (default 0)");
            LOG_INFO(TAG, "  --zlg-abaud      arbitration bitrate, bps (default %u)",
                     (unsigned)HAL_CAN_ZLG_DEFAULT_ARB_BPS);
            LOG_INFO(TAG, "  --zlg-dbaud      data-phase bitrate,   bps (default %u)",
                     (unsigned)HAL_CAN_ZLG_DEFAULT_DATA_BPS);
            LOG_INFO(TAG, "  --node <id>      CANopen node id (1..127, default: %u)",
                     (unsigned)DRIVE_NODE_ID);
            return 0;
        }
    }
    /* The flag handlers above ensure at most one of use_udp/use_zlg/
     * use_vbus is set; defensively guard against future regressions. */
    int n_set = (int)g_app.use_udp + (int)g_app.use_zlg + (int)g_app.use_vbus;
    if(n_set > 1){
        LOG_ERR(TAG, "--zlg / --udp / --vbus are mutually exclusive");
        return 2;
    }

    // --- virtual motor ---
    vmotor_init(&g_vmotor);

    // --- cia402 drive profile ---
    g_app.drive = cia402_create();
    cia402_set_state_change_cb(g_app.drive, on_cia402_state, NULL);
    cia402_set_mode_change_cb (g_app.drive, on_cia402_mode,  NULL);
    cia402_set_supported_modes(g_app.drive,
        CIA402_SUPPORTS_PROFILE_POSITION |
        CIA402_SUPPORTS_PROFILE_VELOCITY |
        CIA402_SUPPORTS_PROFILE_TORQUE   |
        CIA402_SUPPORTS_CYCLIC_SYNC_POSITION |
        CIA402_SUPPORTS_CYCLIC_SYNC_VELOCITY |
        CIA402_SUPPORTS_CYCLIC_SYNC_TORQUE);

    // --- drive OD wraps the cia402 object table + a 0x2000 node-id entry ---
    g_app.node_id = cli_node_id;
    if(build_merged_od(&g_app) != 0){
        LOG_ERR(TAG, "build_merged_od failed");
        return 1;
    }

    // --- CAN backend: ZLG (default), UDP-multicast, or in-process vbus ---
    hal_can_t* h_srv = NULL;
    hal_can_t* h_pdo = NULL;
    hal_can_t* h_cli = NULL;
    if(g_app.use_udp){
#ifdef USE_LINUX_CAN
        // Two endpoints on the same multicast group. Each receives every
        // frame on the "bus" (with sender-tag self-echo filtering), so the
        // USDO server and PDO each get their own subscriber without needing
        // an in-process multiplexer.
        h_srv = hal_can_udp_create(g_app.udp_group, g_app.udp_port);
        h_pdo = hal_can_udp_create(g_app.udp_group, g_app.udp_port);
        if(!h_srv || !h_pdo){
            LOG_ERR(TAG, "hal_can_udp_create failed (loopback up?)");
            return 1;
        }
        // External master drives SDO; in-process master client disabled.
#else
        LOG_ERR(TAG, "--udp requires a Linux build (PLATFORM_LINUX=ON)");
        return 1;
#endif
    }
    else if(g_app.use_zlg){
#ifdef USE_LINUX_CAN
        // Two endpoints share one ZLG channel via internal refcount;
        // each sees the same wire traffic but keeps its own RX callback
        // (USDO-server vs PDO). External master required.
        h_srv = hal_can_zlg_create(&g_app.zlg_cfg);
        h_pdo = hal_can_zlg_create(&g_app.zlg_cfg);
        if(!h_srv || !h_pdo){
            LOG_ERR(TAG, "hal_can_zlg_create failed (USBCANFD plugged in / driver loaded?)");
            return 1;
        }
#else
        LOG_ERR(TAG, "--zlg requires a Linux build (PLATFORM_LINUX=ON)");
        return 1;
#endif
    }
    else {
        vbus_init(&g_app.bus);
        h_srv = vbus_attach(&g_app.bus, "drive.srv");
        h_pdo = vbus_attach(&g_app.bus, "drive.pdo");
        h_cli = vbus_attach(&g_app.bus, "master.cli");
    }

    g_app.hcan_srv = h_srv;
    g_app.hcan_pdo = h_pdo;

    g_app.srv = co_fd_usdo_server_create(h_srv, &g_app.drive_od, g_app.node_id);
    g_app.pdo = can_fd_pdo_create(h_pdo, &g_app.drive_od);
    can_fd_pdo_set_sync_cob_id(g_app.pdo, 0x80);

    // Standard PDO1 mapping (CiA 402 §7.4.5 defaults-ish):
    //   RPDO1 (0x200+id) <- Controlword (0x6040) + TargetPosition (0x607A)
    //   TPDO1 (0x180+id) -> Statusword (0x6041) + PositionActual (0x6064)
    reconfigure_default_pdos(&g_app, g_app.node_id);

    g_app.cli = h_cli ? co_fd_usdo_client_create(h_cli, &g_app.drive_od) : NULL;

    // --- shell ---
    tinysh_set_prompt("sim> ");
    tinysh_add_command(&cmdtab_cmd_status);
    tinysh_add_command(&cmdtab_cmd_report);
    tinysh_add_command(&cmdtab_cmd_shutdown);
    tinysh_add_command(&cmdtab_cmd_switchon);
    tinysh_add_command(&cmdtab_cmd_enable);
    tinysh_add_command(&cmdtab_cmd_disable);
    tinysh_add_command(&cmdtab_cmd_quickstop);
    tinysh_add_command(&cmdtab_cmd_reset);
    tinysh_add_command(&cmdtab_cmd_mode);
    tinysh_add_command(&cmdtab_cmd_target);
    tinysh_add_command(&cmdtab_cmd_load);
    tinysh_add_command(&cmdtab_cmd_vm_max_vel);
    tinysh_add_command(&cmdtab_cmd_vm_max_accel);
    tinysh_add_command(&cmdtab_cmd_vm_pos_kp);
    tinysh_add_command(&cmdtab_cmd_vm_inertia);
    tinysh_add_command(&cmdtab_cmd_vm_reset);
    tinysh_add_command(&cmdtab_cmd_fault);
    tinysh_add_command(&cmdtab_cmd_sdo_get);
    tinysh_add_command(&cmdtab_cmd_sdo_put);
    tinysh_add_command(&cmdtab_cmd_quit);

    /* Porting layer: logger sink (stderr on host), shell-input pump
     * (stdin on host), log level. One init call replaces the mix of
     * tty_raw + select + read + stderr printing this file used to do. */
    {
        platform_porting_cfg_t pcfg = platform_porting_default_cfg_linux();
        /* We own our CAN handles below; null the default one out so
         * platform_porting_shutdown doesn't touch them. */
        pcfg.can               = NULL;
        pcfg.initial_log_level = LOG_LEVEL_INFO;
        platform_porting_init(&pcfg);
    }

    LOG_INFO(TAG, "===============================================");
    LOG_INFO(TAG, "cia402_drive_sim  -  Node %u (kinematic drive simulator)",
             (unsigned)g_app.node_id);
    LOG_INFO(TAG, "vmotor: J=%.2e kgm^2  max_vel=%.0f rad/s  max_accel=%.0f rad/s^2",
             g_vmotor.inertia_kgm2, g_vmotor.max_vel, g_vmotor.max_accel);
    if(g_app.use_udp){
        LOG_INFO(TAG, "CAN backend: UDP loopback, mcast %s:%u",
                 g_app.udp_group ? g_app.udp_group : HAL_CAN_UDP_DEFAULT_GROUP,
                 g_app.udp_port  ? g_app.udp_port  : (uint16_t)HAL_CAN_UDP_DEFAULT_PORT);
        LOG_INFO(TAG, "External master required");
        LOG_INFO(TAG, "Run:  motor_drive_master --udp --first-id %u --count 1",
                 (unsigned)g_app.node_id);
    } else if(g_app.use_zlg){
        LOG_INFO(TAG, "CAN backend: ZLG USBCANFD type=%u dev=%u ch=%u  %u/%u bps",
                 (unsigned)g_app.zlg_cfg.device_type,
                 (unsigned)g_app.zlg_cfg.device_index,
                 (unsigned)g_app.zlg_cfg.channel_index,
                 (unsigned)g_app.zlg_cfg.arb_baud_bps,
                 (unsigned)g_app.zlg_cfg.data_baud_bps);
        LOG_INFO(TAG, "External master required");
        LOG_INFO(TAG, "Run:  motor_drive_master --zlg --first-id %u --count 1",
                 (unsigned)g_app.node_id);
    } else {
        LOG_INFO(TAG, "CAN backend: in-process virtual bus, master node %u",
                 MASTER_NODE_ID);
        LOG_INFO(TAG, "Bring-up:  shutdown  switchon  mode csp  enable");
        LOG_INFO(TAG, "Remote:    sdo_put 0x6040 0 0x06  (shutdown via SDO)");
    }
    LOG_INFO(TAG, "===============================================");
    tinysh_char_in('\r');

    // --- main loop ---
    // Wire timing model (UDP / ZLG):
    //   * vmotor still ticks at 10 kHz so coast/decay works when the
    //     drive is disabled and physics stays consistent regardless of
    //     the master's RPDO cadence.
    //   * Target adoption (OD -> vmotor.target_*) is gated on RPDO
    //     arrival AND state == OPERATION_ENABLED — strict cyclic-sync.
    //   * TPDO emission is event-driven from RPDO arrivals (1:1 with
    //     the master), plus a 10 Hz heartbeat so the master can read
    //     actuals before issuing any RPDO (e.g. `master> slaves`
    //     before `bringup`).
    while(!g_app.quit){
        /* Platform porting pump: drains the shell RX source into
         * tinysh_char_in. On host this is a non-blocking stdin drain;
         * on MCU it's whatever the UART-RX handler queued since last
         * tick. Replaces the fd_set / select / read loop. */
        platform_porting_pump();

        // cia402 state machine + tick advance.
        cia402_process(g_app.drive);

        /* RPDO -> vmotor target adoption (event-driven). The cb in
         * on_rpdo_received set rpdo_pending; the OD has by now been
         * scattered with the fresh payload inside can_fd_pdo_process()
         * from the *previous* iteration. */
        bool fire_tpdo = false;
        if(g_app.rpdo_pending){
            g_app.rpdo_pending = false;
            if(cia402_get_state(g_app.drive) == CIA402_STATE_OPERATION_ENABLED){
                apply_cia402_targets(&g_app);
                fire_tpdo = true;
            }
            /* If not ENABLED we silently drop the RPDO trigger — no
             * target adoption, no paired TPDO response. The heartbeat
             * below still keeps the master informed. */
        }

        /* Periodic 10 kHz vmotor stepping (unchanged from before).
         * When OPERATION_ENABLED the motor tracks the OD targets;
         * otherwise it coasts with mild damping (see vmotor_step). */
        for(int i = 0; i < 5; i++){
            tick_motor_once(&g_app);
        }
        publish_motor_actuals(&g_app);

#ifdef USE_LINUX_CAN
        if(g_app.use_udp || g_app.use_zlg){
            const uint32_t now_ms = get_tick_count();
            /* 10 Hz heartbeat — only kicks in if the RPDO-driven
             * trigger didn't already fire this iteration. */
            if(!fire_tpdo &&
               (uint32_t)(now_ms - g_app.last_tpdo_ms) >= 100u){
                fire_tpdo = true;
            }
            if(fire_tpdo){
                g_app.last_tpdo_ms = now_ms;
                (void)can_fd_tpdo_trigger(g_app.pdo, 0);
            }
        }
#endif

        // Drain transport sockets / driver buffers (no-op in vbus mode).
#ifdef USE_LINUX_CAN
        if(g_app.use_udp){
            hal_can_udp_poll(g_app.hcan_srv);
            hal_can_udp_poll(g_app.hcan_pdo);
        }
        else if(g_app.use_zlg){
            hal_can_zlg_poll(g_app.hcan_srv);
            hal_can_zlg_poll(g_app.hcan_pdo);
        }
#endif

        can_fd_pdo_process(g_app.pdo, 1000);
        co_fd_usdo_server_process(g_app.srv, 1);
        if(g_app.cli){
            co_fd_usdo_client_process(g_app.cli, 1);
        }

        /* 500 µs pacing. platform_sleep_ms is coarse (1 ms resolution)
         * — good enough to keep the host CPU from spinning; an MCU
         * port typically uses a hardware-timer ISR instead and drops
         * this sleep entirely. */
        platform_sleep_ms(0);
    }

    // --- cleanup ---
    if(g_app.cli){ co_fd_usdo_client_destroy(g_app.cli); }
    co_fd_usdo_server_destroy(g_app.srv);
    can_fd_pdo_destroy(g_app.pdo);
#ifdef USE_LINUX_CAN
    if(g_app.use_udp){
        hal_can_udp_destroy(g_app.hcan_srv);
        hal_can_udp_destroy(g_app.hcan_pdo);
    }
    else if(g_app.use_zlg){
        hal_can_zlg_destroy(g_app.hcan_srv);
        hal_can_zlg_destroy(g_app.hcan_pdo);
    }
#endif
    cia402_destroy(g_app.drive);
    LOG_INFO(TAG, "bye.");
    platform_porting_shutdown();
    return 0;
}
