/**
 * @file    cia402_drive_od.h
 * @brief   Shared OD-extension block for the CiA 402 drive apps.
 *
 * @details
 * The base @c cia402 library installs the 14 entries that DS-402 §7
 * marks as mandatory (0x603F, 0x6040, 0x6041, 0x605A, 0x605E, 0x6060,
 * 0x6061, 0x6064, 0x606C, 0x6071, 0x6077, 0x607A, 0x60FF, 0x6502).
 * Several optional-but-common drive entries are not in that set;
 * @ref motor_drive_cia402.c and @ref cia402_drive_sim.c both need
 * them for testing limits, scaling, and homing from the master:
 *
 *   - @c 0x6072 Max torque                    U16  RW  per-mille of rated
 *   - @c 0x6076 Motor rated torque            U32  RW  mNm
 *   - @c 0x6078 Current actual value          I16  RO  per-mille of rated
 *   - @c 0x607C Home offset                   I32  RW  position increments
 *   - @c 0x607D Software position limit       record
 *       - @c 0x607D.0 number of entries       U8   RO  (= 2)
 *       - @c 0x607D.1 min position limit      I32  RW  inc
 *       - @c 0x607D.2 max position limit      I32  RW  inc
 *   - @c 0x6080 Max motor speed               U32  RW  rpm
 *
 * @ref cia402_drive_od_t owns the backing storage, sub-object table,
 * and one @c co_obj_t per index. Apps stitch @ref cia402_drive_od_objs
 * into their merged OD list alongside the @c cia402 mandatory set and
 * any vendor-specific entries (e.g. the node-id @ 0x2000.01).
 *
 * Reading or writing the values from app code is just direct field
 * access — the SDO server and PDO layer do the OD work via the
 * @ref co_obj_t list.
 */

#ifndef SRC_APP_CIA402_DRIVE_OD_H_
#define SRC_APP_CIA402_DRIVE_OD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "co_od.h"
#include <stdint.h>

/**
 * @brief Number of distinct OD indices managed by this helper.
 *
 * 6 original (0x6072/6076/6078/607C/607D/6080) + Identity (0x1000/1008/
 * 1009/100A/1018) + protection (0x6065/6066) + rated current (0x6075) +
 * motion profile (0x6081/6083/6084/6085) + homing (0x6098/6099/609A) +
 * standard gains (0x60F6/60F9/60FB) + encoder resolution (0x608F) +
 * manufacturer (0x2040 offset, 0x2041 gain, 0x2050 stall, 0x2060 hall,
 * 0x2070 motor profile) = 30.
 */
#define CIA402_DRIVE_OD_COUNT       (32u)

/**
 * @brief Default rated torque in micro-Newton-metres (= 0.5 Nm).
 *
 * Mirrors the @c RATED_TORQUE_NM compile-time constant baked into the
 * apps' Nm <-> per-mille conversion. Keep in sync if you change the
 * scaling there.
 */
#define CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_mNm  (500u)   /* 0.5 Nm */

/**
 * @brief OD extension state: storage + co_obj wiring.
 *
 * Allocate one as a long-lived (e.g. file-static) instance and call
 * @ref cia402_drive_od_init exactly once. The contained @c co_obj_t
 * entries reference internal sub-tables that must outlive the OD,
 * so don't move or copy this struct after init.
 */
typedef struct {
    /* ---------------- backing storage ---------------- */

    /** @brief @c 0x6072 — max torque (per-mille of rated; 1000 == 100 %). */
    uint16_t  max_torque;

    /** @brief @c 0x6076 — motor rated torque (mNm, CiA-402 standard unit). */
    uint32_t  motor_rated_torque_mNm;

    /** @brief @c 0x6078 — current actual value (per-mille of rated, CiA-402). */
    int16_t   current_actual_permille;

    /** @brief @c 0x607C — home offset (position increments, signed). */
    int32_t   home_offset;

    /** @brief @c 0x607D.1 — software min position limit (increments). */
    int32_t   pos_limit_min;

    /** @brief @c 0x607D.2 — software max position limit (increments). */
    int32_t   pos_limit_max;

    /** @brief @c 0x6080 — max motor speed (rpm; or app-defined units). */
    uint32_t  max_motor_speed;

    /* ---- Identity (0x1000 / 0x1008 / 0x1009 / 0x100A / 0x1018) ---- */
    uint32_t  device_type;
    uint32_t  vendor_id;
    uint32_t  product_code;
    uint32_t  revision;
    uint32_t  serial;
    char      dev_name[16];
    char      hw_ver[16];
    char      sw_ver[16];

    /* ---- Protection / rating / profile / homing ---- */
    uint32_t  rated_current_mA;          /* 0x6075 */
    uint32_t  following_err_window;      /* 0x6065 */
    uint16_t  following_err_timeout;     /* 0x6066 */
    uint32_t  profile_velocity;          /* 0x6081 */
    uint32_t  profile_accel;             /* 0x6083 */
    uint32_t  profile_decel;             /* 0x6084 */
    uint32_t  quickstop_decel;           /* 0x6085 */
    int8_t    homing_method;             /* 0x6098 */
    uint32_t  homing_speed_switch;       /* 0x6099:1 */
    uint32_t  homing_speed_zero;         /* 0x6099:2 */
    uint32_t  homing_accel;              /* 0x609A */

    /* ---- Standard control gains (0x60F6 / 0x60F9 / 0x60FB) ---- */
    float     cur_kp, cur_ki;            /* 0x60F6:1/2 */
    float     vel_kp, vel_ki;            /* 0x60F9:1/2 */
    float     pos_kp, pos_ki;            /* 0x60FB:1/2 */

    /* ---- Position encoder resolution (0x608F, like a gearbox) ---- */
    uint32_t  enc_increments;            /* 0x608F:1 */
    uint32_t  enc_motor_revs;            /* 0x608F:2 */

    /* ---- Manufacturer (0x2040/0x2041/0x2050/0x2060/0x2070) ---- */
    float     curr_off_a, curr_off_b, curr_off_c;   /* 0x2040:1..3 */
    float     curr_gain_a, curr_gain_b, curr_gain_c;/* 0x2041:1..3 */
    uint32_t  stall_current_mA;          /* 0x2050:1 */
    uint32_t  stall_time_ms;             /* 0x2050:2 */
    float     hall_offset;               /* 0x2060   */
    uint32_t  prof_type;                 /* 0x2070:1 */
    uint32_t  prof_pole_pair;            /* 0x2070:2 */
    float     prof_rs;                   /* 0x2070:3 */
    float     prof_ls_d;                 /* 0x2070:4 */
    float     prof_ls_q;                 /* 0x2070:5 */
    float     prof_flux;                 /* 0x2070:6 */
    float     prof_inertia;              /* 0x2070:7 */
    int32_t   prof_rated_vol;            /* 0x2070:8 */

    int32_t   vf_freq_mhz;               /* 0x2031:1 open-loop V/f freq  */
    uint32_t  vf_amp_milli;              /* 0x2031:2 open-loop V/f amplitude (milli-pu) */
    uint32_t  vf_bsp_enable;             /* 0x2032:1 BSP V/f enable      */
    int32_t   vf_bsp_freq_mhz;           /* 0x2032:2 BSP V/f freq        */
    uint32_t  vf_bsp_amp_milli;          /* 0x2032:3 BSP V/f amplitude   */

    /* ---------------- internal wiring (do not touch) ---------------- */

    uint8_t      n_607D_subs;           /* const 2; backs 0x607D.0      */
    uint8_t      n_1018_subs;           /* const 4; backs 0x1018.0      */
    uint8_t      n_6099_subs;           /* const 2; backs 0x6099.0      */
    uint8_t      n_gain_subs;           /* const 2; backs 0x60Fx.0      */
    uint8_t      n_608F_subs;           /* const 2; backs 0x608F.0      */
    uint8_t      n_2040_subs;           /* const 3; backs 0x2040.0      */
    uint8_t      n_2041_subs;           /* const 3; backs 0x2041.0      */
    uint8_t      n_2050_subs;           /* const 2; backs 0x2050.0      */
    uint8_t      n_2070_subs;           /* const 8; backs 0x2070.0      */
    uint8_t      n_2031_subs;           /* const 2; backs 0x2031.0      */
    uint8_t      n_2032_subs;           /* const 3; backs 0x2032.0      */
    co_sub_obj_t sub_6072[1];
    co_sub_obj_t sub_6076[1];
    co_sub_obj_t sub_6078[1];
    co_sub_obj_t sub_607C[1];
    co_sub_obj_t sub_607D[3];           /* 0=count, 1=min, 2=max        */
    co_sub_obj_t sub_6080[1];
    co_sub_obj_t sub_1000[1];
    co_sub_obj_t sub_1008[1];
    co_sub_obj_t sub_1009[1];
    co_sub_obj_t sub_100A[1];
    co_sub_obj_t sub_1018[5];           /* 0=count, 1..4 = identity     */
    co_sub_obj_t sub_6075[1];
    co_sub_obj_t sub_6065[1];
    co_sub_obj_t sub_6066[1];
    co_sub_obj_t sub_6081[1];
    co_sub_obj_t sub_6083[1];
    co_sub_obj_t sub_6084[1];
    co_sub_obj_t sub_6085[1];
    co_sub_obj_t sub_6098[1];
    co_sub_obj_t sub_6099[3];           /* 0=count, 1=switch, 2=zero    */
    co_sub_obj_t sub_609A[1];
    co_sub_obj_t sub_60F6[3];           /* 0=count, 1=Kp, 2=Ki          */
    co_sub_obj_t sub_60F9[3];
    co_sub_obj_t sub_60FB[3];
    co_sub_obj_t sub_608F[3];           /* 0=count, 1=inc, 2=motor revs */
    co_sub_obj_t sub_2040[4];           /* 0=count, 1..3 offsets        */
    co_sub_obj_t sub_2041[4];           /* 0=count, 1..3 gains          */
    co_sub_obj_t sub_2050[3];           /* 0=count, 1=stall I, 2=stall t*/
    co_sub_obj_t sub_2060[1];           /* hall offset (VAR)            */
    co_sub_obj_t sub_2070[9];           /* 0=count, 1..8 motor profile  */
    co_sub_obj_t sub_2031[3];           /* 0=count, 1 freq, 2 voltage   */
    co_sub_obj_t sub_2032[4];           /* 0=count, 1 en, 2 freq, 3 amp */
    co_obj_t     objs[CIA402_DRIVE_OD_COUNT];
} cia402_drive_od_t;

/**
 * @brief Populate @p _x with sane defaults and wire its sub-tables.
 *
 * Defaults:
 * - @c max_torque                 = 1000  (100 % of rated)
 * - @c motor_rated_torque_mNm     = @ref CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_mNm
 * - @c current_actual_permille    = 0
 * - @c home_offset                = 0
 * - @c pos_limit_min              = INT32_MIN
 * - @c pos_limit_max              = INT32_MAX
 * - @c max_motor_speed            = UINT32_MAX  (no clamp)
 *
 * After this call, @c _x->objs[] is ready to be appended into a
 * merged OD list.
 */
void cia402_drive_od_init(cia402_drive_od_t* _x);

/**
 * @brief Snap the helper back to its install-time defaults without
 *        re-running the sub-table wiring.
 */
void cia402_drive_od_reset(cia402_drive_od_t* _x);

/**
 * @brief Pointer to the @c co_obj_t array (length
 *        @ref CIA402_DRIVE_OD_COUNT) for inclusion in a merged OD.
 */
co_obj_t* cia402_drive_od_objs(cia402_drive_od_t* _x);

#ifdef __cplusplus
}
#endif

#endif /* SRC_APP_CIA402_DRIVE_OD_H_ */
