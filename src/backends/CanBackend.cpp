#include "CanBackend.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

extern "C" {
#include "master_mgr.h"
#include "motor_drive_interface.h"
#include "motor_drive_cia402_intf.h"
#include "cia402_master.h"
#include "co_fd_usdo.h"
#include "co_fd_usdo_sync.h"
#include "can_fd_pdo.h"
#include "hal_can.h"
#include "hal_can_udp.h"
#include "hal_can_zlg.h"               /* SDK platform_linux: multi-endpoint */
#include "hal_uart.h"
#include "hal_uart_linux.h"
#include "motor_drive_feetech_intf.h"
#include "feetech/feetech_servo.h"
}

#include "DynamixelDriver.hpp"

namespace vrmc {

/* PDO snapshot mutated from the CAN RX context, read from the worker.
 * Guarded by a per-slot mutex (mu); copies are short. */
struct PdoSlot {
    mutable std::mutex      mu;
    uint16_t                statusword = 0;
    int32_t                 position   = 0;   /* raw counts            */
    int32_t                 velocity   = 0;   /* raw counts/s          */
    int16_t                 torque     = 0;   /* per-mille of rated    */
    uint16_t                error_code = 0;
    int16_t                 current    = 0;   /* per-mille of rated I  */
    std::atomic<uint64_t>   rx_count{0};
    std::atomic<bool>       fresh{false};
};

struct CanBackend::Slot
{
    uint8_t                node_id   = 0;
    uint8_t                pdo_slot  = 0;
    cia402_master_t*       master    = nullptr;
    motor_drive_intf_t*    intf      = nullptr;
    co_fd_usdo_sync_ctx_t  sync_ctx {};
    char                   name[32]  = {};
    PdoSlot                pdo;

    /* Per-slot PDO cycle cache.
     *
     * The PDO cycle (CanBackend::cycleStep, driven by a 1 kHz timer in
     * MasterWorker) always sends one RPDO per registered slot per
     * period. The cached @c cycle_controlword + targets are what the
     * RPDO carries. Default after open() is @c cw=0x0000 (Disable
     * Voltage — no-op for FSM, but keeps slave's CAN main loop ticking
     * so its SDO server stays responsive). Bringup flips to @c 0x000F;
     * disable to @c 0x0006; quick-stop to @c 0x0002; fault-reset to
     * @c 0x0080 momentarily. Mutex-serialised against bringup/disable
     * walks in MasterWorker so SDO writes never race with cached cw. */
    std::atomic<uint16_t>  cycle_controlword{0};
    std::atomic<int32_t>   tx_target_pos{0};
    std::atomic<int32_t>   tx_target_vel{0};
    std::atomic<int16_t>   tx_target_torque{0};

    /* Brake = controlword bit 8 (Halt). Stored separately from the
     * cycle controlword so engaging Brake doesn't lose the state-machine
     * bits (0-3, 7) — cycleStep OR's 0x0100 into the outgoing cw when
     * this is true. From the slave's perspective: bit 8 set in mode 1
     * (Profile Position) freezes motion; in mode 3 (Profile Velocity)
     * decelerates with Quick-stop ramp; in mode 4 (Profile Torque) zero
     * torque. Drive stays in OPERATION_ENABLED throughout. */
    std::atomic<bool>      brake_engaged{false};
};

/* TPDO1 from the CiA 402 sim (slave -> master, COB-ID 0x180+id):
 *   [0..1]   statusword       (uint16, little-endian)
 *   [2..5]   position_actual  (int32)
 *   [6..9]   velocity_actual  (int32)
 *   [10..11] torque_actual    (int16, per-mille of rated)
 *   [12..13] error_code       (uint16)
 *   [14..15] current_actual   (int16, per-mille of rated, optional)
 * Layout copied from motor_drive_master.c's MASTER_PDO_TPDO_BYTES path.
 */
static constexpr uint8_t kTPdoBytes = 14;

/* RPDO1 from master to sim (master -> slave, COB-ID 0x200+id):
 *   [0..1]   controlword      (uint16, little-endian)
 *   [2..5]   target_position  (int32)
 *   [6..9]   target_velocity  (int32)
 *   [10..11] target_torque    (int16)
 * Layout matches motor_drive_master.c's MASTER_PDO_RPDO_BYTES + the
 * sim's RPDO1 mapping registered in cia402_drive_sim.c. The diagnostic
 * sends this once per MasterWorker tick so the sim's event-driven TPDO
 * response fires alongside its 10 Hz heartbeat — see CanBackend::pdoSend.
 */
static constexpr uint8_t kRPdoBytes = 12;

static uint16_t le16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
static uint32_t le32(const uint8_t* p) {
    return uint32_t(p[0])       | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

static void pack_le16(uint8_t* p, uint16_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
}
static void pack_le32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v);        p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16);  p[3] = uint8_t(v >> 24);
}

/* Static RPDO callback. arg is a PdoSlot* (file-scope struct, not the
 * private CanBackend::Slot). Runs from the CAN RX context — background
 * poll thread for ZLG, hal_can_udp_poll caller for UDP. */
static void rpdoRx(uint32_t /*cob_id*/, const uint8_t* data,
                   uint8_t len, void* arg)
{
    auto* p =  static_cast<PdoSlot*>(arg);
    if (!p || !data || len < kTPdoBytes ){ return; }
    std::lock_guard<std::mutex> lk(p->mu);
    p->statusword = le16(data + 0);
    p->position   = static_cast<int32_t>(le32(data + 2));
    p->velocity   = static_cast<int32_t>(le32(data + 6));
    p->torque     = static_cast<int16_t>(le16(data + 10));
    p->error_code = le16(data + 12);
    /* current_actual (0x6078) is appended at [14..15] when the drive's
     * TPDO carries it (16-byte frame); older 14-byte frames lack it. */
    p->current    = (len >= 16) ? static_cast<int16_t>(le16(data + 14)) : 0;
    p->rx_count.fetch_add(1, std::memory_order_relaxed);
    p->fresh.store(true, std::memory_order_release);
}

/* Static pump hook passed to every slot's sync_ctx. Lets the sync SDO
 * helper drain the USDO client while a blocking read/write waits. */
static void pumpStub(void* arg)
{
    auto* self = static_cast<CanBackend*>(arg);
    if (self){
        self->pump();
    }
}

CanBackend::CanBackend() = default;

CanBackend::~CanBackend() { close(); }

bool CanBackend::open(master_mgr_t* mgr, const CanConfig& cfg, QString* err)
{
    if (!mgr){
        if (err){ *err = QStringLiteral("master_mgr is null"); }
        return false;
    }
    if (cfg.count == 0){
        if (err){ *err = QStringLiteral("slave count is 0"); }
        return false;
    }

    /* Build the requested transport. CAN paths share the rest of the
     * bring-up (SDO client + CiA 402 per-slave), so we fall through to
     * the common branch below once m_canCli is non-null. UART paths
     * (Feetech / Dynamixel) register slaves directly and return early. */
    m_kind = cfg.kind;
    switch (cfg.kind){
    case CanKind::Udp: {
        const QByteArray group = cfg.group.toUtf8();
        m_canCli = hal_can_udp_create(group.constData(), cfg.port);
        m_canPdo = hal_can_udp_create(group.constData(), cfg.port);
        if (!m_canCli || !m_canPdo){
            if (err){ *err = QStringLiteral("hal_can_udp_create failed"); }
            close();
            return false;
        }
        break;
    }
    case CanKind::Zlg: {
        /* SDK's hal_can_zlg shares a channel between multiple endpoints
         * via internal refcount on (device, channel) tuples, so we can
         * create separate USDO-client and PDO endpoints exactly the way
         * the UDP branch does — and the way motor_drive_master.c does
         * against real hardware. No more "single consumer per channel"
         * limitation that forced PDO off on ZLG. */
        hal_can_zlg_cfg_t zcfg{};
        zcfg.device_type       = HAL_CAN_ZLG_DEFAULT_DEVICE_TYPE;
        zcfg.device_index      = HAL_CAN_ZLG_DEFAULT_DEVICE_INDEX;
        zcfg.channel_index     = cfg.zlgChannel;
        zcfg.arb_baud_bps      = cfg.zlgBitrate;
        zcfg.data_baud_bps     = cfg.zlgFdBitrate;
        zcfg.enable_terminator = 1;
        zcfg.iso_canfd         = 1;
        m_canCli = hal_can_zlg_create(&zcfg);
        m_canPdo = hal_can_zlg_create(&zcfg);
        if (!m_canCli || !m_canPdo){
            if (err){
                *err = QStringLiteral("hal_can_zlg_create failed "
                                      "(libcontrolcanfd.so missing or "
                                      "USB-CANFD device not present?)");
            }
            close();
            return false;
        }
        break;
    }
    case CanKind::Feetech: {
        const QByteArray dev = cfg.uartDevice.toUtf8();
        m_uart = hal_uart_linux_create(dev.constData());
        if (!m_uart){
            if (err){
                *err = QStringLiteral("hal_uart_linux_create failed for %1")
                           .arg(cfg.uartDevice);
            }
            close();
            return false;
        }
        hal_uart_config_t ucfg{};
        ucfg.baudrate     = cfg.uartBaud;
        ucfg.word_bits    = 8;
        ucfg.parity       = HAL_UART_PARITY_NONE;
        ucfg.stop_bits    = HAL_UART_STOP_1;
        ucfg.flow_control = false;
        if (hal_uart_init(m_uart, &ucfg) != 0){
            if (err){ *err = QStringLiteral("hal_uart_init failed"); }
            close();
            return false;
        }
        feetech_servo_bind(&m_feetechBus, m_uart);
        /* Register one motor_drive_feetech_intf per servo id. */
        for (uint8_t i = 0; i < cfg.count; ++i){
            auto* slot    = new Slot();
            slot->node_id = uint8_t(cfg.first_id + i);
            std::snprintf(slot->name, sizeof(slot->name),
                          "feetech-%u", unsigned(slot->node_id));
            motor_drive_feetech_intf_cfg_t icfg{};
            icfg.bus                 = &m_feetechBus;
            icfg.id                  = slot->node_id;
            icfg.counts_per_rad      = 4096.0f / (2.0f * 3.14159265f);
            icfg.steps_per_rad_per_s = 0.0f;
            icfg.A_per_Nm            = 0.0f;
            icfg.acc_default         = 50;
            icfg.speed_default       = 1000;
            icfg.bringup_mode        = MOTOR_INTF_MODE_POSITION;
            icfg.name                = slot->name;
            slot->intf = motor_drive_feetech_intf_create(&icfg);
            if (!slot->intf){
                if (err){ *err = QStringLiteral("feetech intf create failed"); }
                delete slot; close(); return false;
            }
            if (master_mgr_add_slave(mgr, slot->intf) < 0){
                if (err){ *err = QStringLiteral("master_mgr_add_slave failed"); }
                motor_drive_intf_free(slot->intf);
                delete slot; close(); return false;
            }
            m_slots.push_back(slot);
        }
        return true;   /* UART path: no SDO / PDO bring-up needed. */
    }
    case CanKind::Dynamixel: {
        /* In-tree Dynamixel Protocol 2.0 driver (DynamixelDriver.cpp).
         * Opens the UART, runs the CRC/framing/stuffing itself — no
         * ROBOTIS SDK dependency, so this just works at runtime with no
         * compile flags. cfg.dxlProtocolVer is accepted for forward
         * compatibility but only 2.0 is implemented today. */
        std::string dxlErr;
        m_dxlBus = DynamixelBus::open(cfg.uartDevice.toStdString(),
                                       cfg.uartBaud, &dxlErr);
        if (!m_dxlBus || !m_dxlBus->ok()){
            if (err){
                *err = QStringLiteral("Dynamixel bus open failed: %1")
                           .arg(QString::fromStdString(dxlErr));
            }
            close();
            return false;
        }
        for (uint8_t i = 0; i < cfg.count; ++i){
            auto* slot    = new Slot();
            slot->node_id = uint8_t(cfg.first_id + i);
            std::snprintf(slot->name, sizeof(slot->name),
                          "dxl-%u", unsigned(slot->node_id));
            DynamixelIntfCfg icfg{};
            icfg.bus          = m_dxlBus;
            icfg.id           = slot->node_id;
            icfg.A_per_Nm     = 0.0f;
            icfg.bringup_mode = MOTOR_INTF_MODE_POSITION;
            icfg.name         = slot->name;
            slot->intf = makeDynamixelIntf(icfg);
            if (!slot->intf){
                if (err){ *err = QStringLiteral("dynamixel intf create failed"); }
                delete slot; close(); return false;
            }
            if (master_mgr_add_slave(mgr, slot->intf) < 0){
                if (err){ *err = QStringLiteral("master_mgr_add_slave failed"); }
                motor_drive_intf_free(slot->intf);
                delete slot; close(); return false;
            }
            m_slots.push_back(slot);
        }
        return true;
    }
    }

    m_client = co_fd_usdo_client_create(m_canCli, /*od=*/nullptr);
    if (!m_client){
        if (err){ *err = QStringLiteral("co_fd_usdo_client_create failed"); }
        close();
        return false;
    }

    m_slots.reserve(cfg.count);
    for (uint8_t i = 0; i < cfg.count; ++i){
        auto* slot  = new Slot();
        slot->node_id = static_cast<uint8_t>(cfg.first_id + i);

        slot->sync_ctx.client     = m_client;
        slot->sync_ctx.node_id    = slot->node_id;
        slot->sync_ctx.timeout_ms = cfg.sdo_timeout_ms;
        slot->sync_ctx.pump       = pumpStub;
        slot->sync_ctx.pump_arg   = this;

        cia402_master_cfg_t mcfg{};
        mcfg.read   = co_fd_usdo_sync_read;
        mcfg.write  = co_fd_usdo_sync_write;
        mcfg.io_arg = &slot->sync_ctx;

        slot->master = cia402_master_create(&mcfg);
        if (!slot->master){
            if (err){ *err = QStringLiteral("cia402_master_create failed"); }
            delete slot;
            close();
            return false;
        }

        std::snprintf(slot->name, sizeof(slot->name),
                      "cia402-node-%u", (unsigned)slot->node_id);

        motor_drive_cia402_intf_cfg_t icfg{};
        icfg.master          = slot->master;
        icfg.scale           = nullptr;
        icfg.gain_map        = nullptr;
        icfg.name            = slot->name;
        icfg.routing_node_id = &slot->sync_ctx.node_id;
        icfg.initial_node_id = slot->node_id;

        slot->intf = motor_drive_cia402_intf_create(&icfg);
        if (!slot->intf){
            if (err){ *err = QStringLiteral("motor_drive_cia402_intf_create failed"); }
            cia402_master_destroy(slot->master);
            delete slot;
            close();
            return false;
        }
        if (master_mgr_add_slave(mgr, slot->intf) < 0){
            if (err){ *err = QStringLiteral("master_mgr_add_slave failed"); }
            motor_drive_intf_free(slot->intf);
            cia402_master_destroy(slot->master);
            delete slot;
            close();
            return false;
        }
        m_slots.push_back(slot);
    }

    /* Subscribe to each slave's TPDO1 (slave -> master). The simulator
     * emits at 2 kHz; we cache the latest frame per slot and let the
     * worker poll the cache at its UI tick rate. PDO uses its own
     * hal_can endpoint so it doesn't steal the USDO client's RX cb. */
    if (!m_canPdo){
        /* Only reachable for UART transports (Feetech/Dynamixel), which
         * don't allocate a CAN endpoint and don't have PDOs. CAN paths
         * (UDP / ZLG) both create m_canPdo above, so they fall through
         * to the per-slave RPDO subscription below. */
        return true;
    }
    m_pdo = can_fd_pdo_create(m_canPdo, /*od=*/nullptr);
    if (!m_pdo){
        if (err){ *err = QStringLiteral("can_fd_pdo_create failed"); }
        close();
        return false;
    }
    for (uint8_t i = 0; i < cfg.count; ++i){
        Slot* slot = m_slots[i];
        slot->pdo_slot = i;

        /* Outgoing master -> slave RPDO (cob 0x200+id, 12 B). Mirrors
         * motor_drive_master.c:710-717. Even if the diagnostic doesn't
         * actively scrub setpoints at the rate of a real controller,
         * registering the TPDO slot is what lets the PDO state machine
         * pair RxPDO callbacks with their TxPDO siblings and advance
         * cleanly on each cycle. */
        co_tpdo_config_t tcfg{};
        tcfg.cob_id             = 0x200u + slot->node_id;
        tcfg.trans_type         = CO_PDO_TX_ASYNC_MFR;
        tcfg.data_len           = kRPdoBytes;
        tcfg.inhibit_time_100us = 0;
        tcfg.event_timer_ms     = 0;
        if (can_fd_tpdo_configure(m_pdo, slot->pdo_slot, &tcfg) != 0){
            if (err){
                *err = QStringLiteral("can_fd_tpdo_configure failed for slot %1")
                           .arg(i);
            }
            close();
            return false;
        }

        /* Incoming slave -> master TPDO (cob 0x180+id, 14 B). */
        co_rpdo_config_t rcfg{};
        rcfg.cob_id     = 0x180u + slot->node_id;
        rcfg.trans_type = CO_PDO_TX_ASYNC_MFR;
        rcfg.data_len   = kTPdoBytes;
        rcfg.cb         = rpdoRx;
        rcfg.arg        = &slot->pdo;
        if (can_fd_rpdo_configure(m_pdo, slot->pdo_slot, &rcfg) != 0){
            if (err){
                *err = QStringLiteral("can_fd_rpdo_configure failed for slot %1")
                           .arg(i);
            }
            close();
            return false;
        }
    }

    /* Probe phase: run the PDO cycle for ~250 ms and wait for every
     * slave to emit at least one TPDO. Each cycleStep sends one RPDO
     * per slot (cw=0x0000 from the cache, no-op for DS402) which on
     * a frame-driven board firmware is what wakes the slave's main
     * loop so it can pair a TPDO + start serving SDO. Mirrors what
     * motor_drive_master.c's cycle_run_loop does from cycle 0 — no
     * special probe code, just the normal cycle running.
     */
    {
        /* A node we expect to be offline (bootloader / SDO-only) never
         * sends a heartbeat, so don't burn the full second waiting — a
         * short look is enough to catch one if it IS live, then proceed. */
        const int     kProbeTimeoutMs = cfg.allow_offline ? 150 : 1000;
        constexpr int kSliceMs        = 5;
        const int     slices          = kProbeTimeoutMs / kSliceMs;
        for (int s = 0; s < slices; ++s){
            cycleStep();
            pump();
            bool all_seen = true;
            for (Slot* slot : m_slots){
                if (!slot->pdo.fresh.load(std::memory_order_acquire)){
                    all_seen = false;
                    break;
                }
            }
            if (all_seen){ break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(kSliceMs));
        }
        int count = 0;
        for (Slot* slot : m_slots){
            if (slot->pdo.fresh.load(std::memory_order_acquire)){
                count++;
                continue;
            }
        }

        if (!count){
            if (cfg.allow_offline){
                /* Bootloader / offline target: the slaves are already
                 * registered with master_mgr from the bring-up loop above;
                 * we just didn't see a heartbeat. Proceed so the firmware-
                 * upgrade path (which only needs the node id + CAN handle)
                 * can run. */
                if (err){
                    *err = QStringLiteral("connected with no TPDO heartbeat "
                                          "(allow_offline) — bootloader / "
                                          "offline node");
                }
                return true;
            }
            if (err){
                *err = QStringLiteral(
                           "no slave responded at node  within %2 ms "
                           "(no TPDO after %3 RPDO cycles). Check that the "
                           "drive is powered + on the correct bitrate.")
                           .arg(kProbeTimeoutMs)
                           .arg(slices);
            }
            close();
            return false;
        }
    }
    return true;
}

void CanBackend::close()
{
    /* master_mgr_destroy() frees the motor_drive_intf_t*s it owns; we
     * still own the cia402_master_t* handles and the transport. */
    for (auto* slot : m_slots){
        if (slot->master){ cia402_master_destroy(slot->master); }
        delete slot;
    }
    m_slots.clear();

    if (m_pdo)   { can_fd_pdo_destroy(m_pdo);            m_pdo    = nullptr; }
    if (m_client){ co_fd_usdo_client_destroy(m_client); m_client = nullptr; }

    auto destroyHalCan = [this](hal_can_t** h){
        if (!*h){ return; }
        switch (m_kind){
        case CanKind::Udp: hal_can_udp_destroy(*h); break;
        case CanKind::Zlg: hal_can_zlg_destroy(*h); break;
        case CanKind::Feetech:
        case CanKind::Dynamixel:
            /* UART transports never allocate hal_can handles. */
            break;
        }
        *h = nullptr;
    };
    destroyHalCan(&m_canPdo);
    destroyHalCan(&m_canCli);

    /* UART teardown. feetech_servo_bind stores a raw pointer; unbind so
     * the servo helper doesn't hold on to stale memory. */
    if (m_kind == CanKind::Feetech){
        feetech_servo_unbind(&m_feetechBus);
    }
    if (m_dxlBus){ delete m_dxlBus; m_dxlBus = nullptr; }
    if (m_uart){
        hal_uart_linux_destroy(m_uart);
        m_uart = nullptr;
    }
}

void CanBackend::pump()
{
    /* Both UDP and the SDK's ZLG backend deliver frames only when
     * polled (the SDK's hal_can_zlg has no background thread — same
     * cadence model as hal_can_udp). Poll both endpoints so the
     * USDO-client and PDO socket each drain. Then the PDO + USDO
     * state machines tick regardless of transport. */
    if (m_kind == CanKind::Udp){
        if (m_canCli){ hal_can_udp_poll(m_canCli); }
        if (m_canPdo){ hal_can_udp_poll(m_canPdo); }
    } else if (m_kind == CanKind::Zlg){
        /* Both endpoints share the same physical ZLG channel via the
         * SDK's refcounted (device, channel) tuple. hal_can_zlg_poll
         * fans frames out to every endpoint registered on the channel,
         * so a single poll drains for both — calling it twice just
         * burns a ZCAN_GetReceiveNum round-trip on each idle queue. */
        if (m_canPdo){ hal_can_zlg_poll(m_canPdo); }
    }
    if (m_pdo)   { can_fd_pdo_process(m_pdo, /*dt_us=*/1000); }
    if (m_client){ co_fd_usdo_client_process(m_client, 1); }
    /* Feetech / Dynamixel: no async RX to pump — every servo op is a
     * synchronous request/reply done inside motor_drive_*_intf. */
}

void CanBackend::pumpForBoot()
{
    /* Drain only the boot endpoint (m_canPdo, where boot_output_cofd_create
     * put its USDO client). Deliberately does NOT poll m_canCli or run
     * can_fd_pdo_process / co_fd_usdo_client_process(m_client): during an
     * upgrade the normal stack must stay dormant so it never answers / aborts
     * the boot transfer's frames on the shared bus. The boot client itself is
     * serviced by boot_master_process() in the caller. */
    if (m_kind == CanKind::Udp){
        if (m_canPdo){ hal_can_udp_poll(m_canPdo); }
    } else if (m_kind == CanKind::Zlg){
        if (m_canPdo){ hal_can_zlg_poll(m_canPdo); }
    }
}

int CanBackend::writePdoMapping(int slotIdx, bool isTpdo,
                                const std::vector<PdoMapEntry>& entries,
                                QString* err)
{
    if (m_kind != CanKind::Udp && m_kind != CanKind::Zlg){
        if (err){
            *err = QStringLiteral("PDO mapping is CiA 402-only; "
                                  "current transport is Feetech/Dynamixel");
        }
        return -1;
    }
    if (slotIdx < 0 || static_cast<size_t>(slotIdx) >= m_slots.size()){
        if (err){ *err = QStringLiteral("invalid slave index"); }
        return -1;
    }
    if (entries.size() > 16){
        if (err){ *err = QStringLiteral("too many mapping entries (max 16)"); }
        return -1;
    }
    /* SDO helpers need an active USDO client — no-op if we're closed. */
    if (!m_client){
        if (err){ *err = QStringLiteral("not connected"); }
        return -1;
    }

    Slot*    slot    = m_slots[slotIdx];
    const uint16_t commIdx = isTpdo ? 0x1800u : 0x1400u;
    const uint16_t mapIdx  = isTpdo ? 0x1A00u : 0x1600u;
    const uint32_t cobId   = (isTpdo ? 0x180u : 0x200u) + slot->node_id;

    auto writeU32 = [&](uint16_t idx, uint8_t sub, uint32_t v) -> int {
        uint8_t buf[4] = { uint8_t(v), uint8_t(v >> 8),
                           uint8_t(v >> 16), uint8_t(v >> 24) };
        return co_fd_usdo_sync_write(idx, sub, buf, 4, &slot->sync_ctx);
    };
    auto writeU8 = [&](uint16_t idx, uint8_t sub, uint8_t v) -> int {
        return co_fd_usdo_sync_write(idx, sub, &v, 1, &slot->sync_ctx);
    };

    int rc = writeU32(commIdx, 1, cobId | 0x80000000u);
    if (rc != 0){
        if (err){ *err = QStringLiteral("disable PDO failed (rc=%1)").arg(rc); }
        return rc;
    }
    rc = writeU8(mapIdx, 0, 0);
    if (rc != 0){
        if (err){ *err = QStringLiteral("clear map count failed (rc=%1)").arg(rc); }
        (void)writeU32(commIdx, 1, cobId);     /* best-effort re-enable */
        return rc;
    }
    for (size_t i = 0; i < entries.size(); ++i){
        const auto& e = entries[i];
        const uint32_t packed =
            (uint32_t(e.idx) << 16) | (uint32_t(e.sub) << 8) | e.bits;
        rc = writeU32(mapIdx, uint8_t(i + 1), packed);
        if (rc != 0){
            if (err){
                *err = QStringLiteral("map entry %1 failed (rc=%2)")
                           .arg(i + 1).arg(rc);
            }
            (void)writeU32(commIdx, 1, cobId);
            return rc;
        }
    }
    rc = writeU8(mapIdx, 0, uint8_t(entries.size()));
    if (rc != 0){
        if (err){ *err = QStringLiteral("set map count failed (rc=%1)").arg(rc); }
        (void)writeU32(commIdx, 1, cobId);
        return rc;
    }
    rc = writeU32(commIdx, 1, cobId);
    if (rc != 0){
        if (err){ *err = QStringLiteral("enable PDO failed (rc=%1)").arg(rc); }
        return rc;
    }
    return 0;
}

int CanBackend::readSdo(int slotIdx, uint16_t odIdx, uint8_t odSub,
                        void* buf, uint32_t len, QString* err)
{
    if (m_kind != CanKind::Udp && m_kind != CanKind::Zlg){
        if (err){
            *err = QStringLiteral("SDO R/W is CiA 402-only");
        }
        return -1;
    }
    if (slotIdx < 0 || static_cast<size_t>(slotIdx) >= m_slots.size() || !buf){
        if (err){ *err = QStringLiteral("invalid args"); }
        return -1;
    }
    Slot* slot = m_slots[slotIdx];
    const int rc = co_fd_usdo_sync_read(odIdx, odSub, buf, len, &slot->sync_ctx);
    if (rc != 0 && err){
        *err = QStringLiteral("SDO read 0x%1.%2 failed (rc=%3)")
                   .arg(odIdx, 4, 16, QChar('0')).arg(odSub).arg(rc);
    }
    return rc;
}

int CanBackend::writeSdo(int slotIdx, uint16_t odIdx, uint8_t odSub,
                         const void* buf, uint32_t len, QString* err)
{
    if (m_kind != CanKind::Udp && m_kind != CanKind::Zlg){
        if (err){
            *err = QStringLiteral("SDO R/W is CiA 402-only");
        }
        return -1;
    }
    if (slotIdx < 0 || static_cast<size_t>(slotIdx) >= m_slots.size() || !buf){
        if (err){ *err = QStringLiteral("invalid args"); }
        return -1;
    }
    Slot* slot = m_slots[slotIdx];
    const int rc = co_fd_usdo_sync_write(odIdx, odSub, buf, len, &slot->sync_ctx);
    if (rc != 0 && err){
        *err = QStringLiteral("SDO write 0x%1.%2 failed (rc=%3)")
                   .arg(odIdx, 4, 16, QChar('0')).arg(odSub).arg(rc);
    }
    return rc;
}

bool CanBackend::getPdo(uint32_t idx, PdoFrame* out) const
{
    if (!out || idx >= m_slots.size()){ return false; }
    const Slot* slot = m_slots[idx];
    if (!slot->pdo.fresh.load(std::memory_order_acquire)){
        out->fresh = false;
        return false;
    }
    std::lock_guard<std::mutex> lk(slot->pdo.mu);
    out->statusword = slot->pdo.statusword;
    out->position   = slot->pdo.position;
    out->velocity   = slot->pdo.velocity;
    out->torque     = slot->pdo.torque;
    out->error_code = slot->pdo.error_code;
    out->current    = slot->pdo.current;
    out->rx_count   = slot->pdo.rx_count.load(std::memory_order_relaxed);
    out->fresh      = true;
    return true;
}

int CanBackend::pdoSend(uint32_t idx, uint16_t controlword,
                        int32_t target_pos, int32_t target_vel,
                        int16_t target_torque)
{
    if (!m_pdo || idx >= m_slots.size()){ return -1; }
    Slot* slot = m_slots[idx];

    uint8_t buf[kRPdoBytes];
    pack_le16(buf + 0,  controlword);
    pack_le32(buf + 2,  static_cast<uint32_t>(target_pos));
    pack_le32(buf + 6,  static_cast<uint32_t>(target_vel));
    pack_le16(buf + 10, static_cast<uint16_t>(target_torque));

    int rc = can_fd_tpdo_write(m_pdo, slot->pdo_slot, buf, sizeof(buf));
    if (rc != 0){ return rc; }
    return can_fd_tpdo_trigger(m_pdo, slot->pdo_slot);
}

void CanBackend::setCycleControlword(uint32_t idx, uint16_t controlword)
{
    if (idx >= m_slots.size()){ return; }
    m_slots[idx]->cycle_controlword.store(controlword, std::memory_order_release);
}

void CanBackend::setTarget(uint32_t idx, int32_t pos, int32_t vel, int16_t torque)
{
    if (idx >= m_slots.size()){ return; }
    Slot* slot = m_slots[idx];
    slot->tx_target_pos   .store(pos,    std::memory_order_release);
    slot->tx_target_vel   .store(vel,    std::memory_order_release);
    slot->tx_target_torque.store(torque, std::memory_order_release);
}

void CanBackend::cycleStep()
{
    if (!m_pdo){ return; }
    for (Slot* slot : m_slots){
        uint16_t cw = slot->cycle_controlword.load(std::memory_order_acquire);
        if (slot->brake_engaged.load(std::memory_order_acquire)){
            cw |= 0x0100u;   /* CiA-402 controlword bit 8 = Halt */
        }
        (void)pdoSend(static_cast<uint32_t>(slot->pdo_slot), cw,
                      slot->tx_target_pos   .load(std::memory_order_acquire),
                      slot->tx_target_vel   .load(std::memory_order_acquire),
                      slot->tx_target_torque.load(std::memory_order_acquire));
    }
}

void CanBackend::setBrake(uint32_t idx, bool engaged)
{
    if (idx >= m_slots.size()){ return; }
    m_slots[idx]->brake_engaged.store(engaged, std::memory_order_release);
}

bool CanBackend::brake(uint32_t idx) const
{
    if (idx >= m_slots.size()){ return false; }
    return m_slots[idx]->brake_engaged.load(std::memory_order_acquire);
}

}  // namespace vrmc
