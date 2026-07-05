/**
 * @file   CanBackend.hpp
 * @brief  CiA 402-over-UDP-CAN backend wiring for the master manager.
 *
 * Owns the hal_can_udp handles, the USDO client, and the cia402_master
 * per slave. Registers each slave into a master_mgr_t through
 * motor_drive_cia402_intf_create so the UI only talks to the abstract
 * motor_drive_interface.
 *
 * No PDO hot path in V0 — master_mgr_refresh() drives all actuals over
 * SDO. Good enough for loopback demos and the diagnostic UI's refresh
 * rate (10-50 Hz). PDO plumbing is a V1 addition.
 */

#pragma once

#include <QString>
#include <cstdint>
#include <vector>

extern "C" {
#include "hal_can.h"
#include "cia402_master.h"
#include "co_fd_usdo.h"
#include "can_fd_pdo.h"
#include "motor_drive_interface.h"
#include "master_mgr.h"

}

namespace vrmc {

/**
 * @brief Which master-side transport to instantiate.
 *
 * Both kinds are CAN-based CiA 402 drives. The former Feetech / Dynamixel
 * RS-485 servo bus options were removed — this tool targets CANopen /
 * CiA-402 motor drives; RS-485 hobby-servo diagnostics live elsewhere.
 */
enum class CanKind {
    Udp       = 0,   /**< UDP-multicast "CAN" (loopback sim).            */
    Zlg       = 1    /**< ZLG USB-CANFD adapter.                         */
};

struct CanConfig
{
    /* --- Transport selector --- */
    CanKind  kind     = CanKind::Zlg;

    /* --- UDP transport (hal_can_udp) --- */
    QString  group    = "239.192.0.42"; /**< UDP multicast group (matches drive). */
    uint16_t port     = 23400;          /**< UDP port.                    */

    /* --- ZLG USB-CANFD transport (hal_can_zlg) --- */
    QString  zlgLibPath  = "libcontrolcanfd.so"; /**< dlopen() target.   */
    uint32_t zlgChannel  = 0;                    /**< channel index 0/1. */
    uint32_t zlgBitrate  =   500'000;            /**< arb-phase bps.     */
    uint32_t zlgFdBitrate = 2'000'000;           /**< data-phase bps.    */

    /* --- Common --- */
    uint8_t  first_id       = 5;        /**< First slave id.              */
    uint8_t  count          = 1;        /**< Number of slaves to bring up.*/
    uint32_t sdo_timeout_ms = 100;
    /* Register the slaves even if none emits a TPDO heartbeat during the
     * probe (open() warns instead of failing). Used by the firmware-
     * upgrade demo to attach to a bootloader node, which is an SDO server
     * only and never sends PDO. */
    bool     allow_offline  = false;
};

class CanBackend
{
public:
    CanBackend();
    ~CanBackend();

    CanBackend(const CanBackend&)            = delete;
    CanBackend& operator=(const CanBackend&) = delete;

    /** Open the transport + register @c cfg.count slaves into @p mgr. */
    bool open(master_mgr_t* mgr, const CanConfig& cfg, QString* err = nullptr);

    /** Tear everything back down. Idempotent. */
    void close();

    bool isOpen() const { return m_canCli != nullptr; }

    /** Pump the transport once (drain RX + service USDO client + PDO timers). */
    void pump();

    /** Drain ONLY the boot endpoint's transport, without running the
     *  normal USDO-client / PDO state machines. Used during a firmware
     *  upgrade: the boot USDO client (created on @ref canHandle) is driven
     *  by boot_master_process, and the normal client must stay completely
     *  idle so it can't inject an SDO abort onto the shared bus mid-
     *  transfer. Mirrors the dedicated-endpoint model the standalone
     *  bootmaster_node uses. */
    void pumpForBoot();

    /** Latest TPDO snapshot from a slave, raw drive units. */
    struct PdoFrame {
        uint16_t statusword = 0;
        int32_t  position   = 0;     /**< raw counts                     */
        int32_t  velocity   = 0;     /**< raw counts/s                   */
        int16_t  torque     = 0;     /**< per-mille of rated torque      */
        uint16_t error_code = 0;
        int16_t  current    = 0;     /**< per-mille of rated current     */
        uint64_t rx_count   = 0;     /**< running PDO RX counter         */
        bool     fresh      = false; /**< true if any PDO yet seen       */
    };

    /** Atomically copy out a slave's most recent PDO snapshot. */
    bool getPdo(uint32_t idx, PdoFrame* out) const;

    /** @return true when the PDO layer is wired up for the current
     *  transport (UDP / ZLG). Worker uses this to decide whether
     *  telemetry should come from cached TPDO frames (fast, passive)
     *  or from per-tick SDO reads (slow, blast-y). */
    bool hasPdo() const { return m_pdo != nullptr; }

    /** Raw hal_can endpoint (the polled channel) for an out-of-band
     *  CANopen-FD client such as the bootloader's USDO transport
     *  (boot_output_cofd_create). The caller must serialise its use
     *  with the normal PDO/SDO traffic — the worker pauses the PDO cycle
     *  for the duration of a firmware upgrade. */
    void* canHandle() const { return reinterpret_cast<void*>(m_canPdo); }

    /** @brief Send one outgoing RPDO frame to a slave.
     *
     *  Builds the 12-byte payload `[controlword | target_pos |
     *  target_vel | target_torque]` (little-endian), then
     *  @c can_fd_tpdo_write + @c can_fd_tpdo_trigger. Mirrors
     *  @c cia402_pdo_send in vr-mc-sdk/app/motor_drive_master.c.
     *
     *  @return 0 on success; -1 if the slot is invalid or PDO is
     *          unavailable (UART transports); transport-level error
     *          codes pass through from @c can_fd_tpdo_*. */
    int pdoSend(uint32_t idx, uint16_t controlword,
                int32_t target_pos, int32_t target_vel,
                int16_t target_torque);

    /** @brief Set the controlword the PDO cycle streams to a slot.
     *
     *  Every registered slot is always part of the cycle — the cycle
     *  sends one RPDO per slot per period. This call just updates the
     *  cached controlword the next RPDO carries:
     *
     *    - @c 0x0000 Disable Voltage     — unarmed / pre-bringup / post-disable
     *    - @c 0x0006 Shutdown            — disable walk target
     *    - @c 0x000F Enable Operation    — armed (post-bringup, motors hot)
     *    - @c 0x0002 Quick Stop          — controlled decel
     *    - @c 0x0080 Fault Reset         — rising edge to clear faults
     *
     *  Mirrors @c motor_drive_master.c's `cia402_slots[i].pdo.controlword`
     *  cache. Safe to call from any thread (atomic store); the cycle
     *  picks it up on the next period.
     */
    void setCycleControlword(uint32_t idx, uint16_t controlword);

    /** @brief Update a slot's cached PDO targets. Safe to call from
     *  any thread; values are read by the next cycle period. */
    void setTarget(uint32_t idx, int32_t pos, int32_t vel, int16_t torque);

    /** @brief Engage / release the Halt bit (controlword bit 8) for a
     *  slot. Composed with the cached state-machine cw inside
     *  @ref cycleStep so engaging Brake doesn't drop OPERATION_ENABLED.
     *  When engaged, the slave decelerates per its mode (mode-specific
     *  behavior; see CiA-402 §6.10.6) and holds zero motion. */
    void setBrake(uint32_t idx, bool engaged);

    /** @return current Brake state for a slot. */
    bool brake(uint32_t idx) const;

    /** @brief Run one period of the PDO cycle: send one RPDO per
     *  registered slot using its cached controlword + targets.
     *
     *  Driven by a dedicated 1 kHz timer in @c MasterWorker. Matches
     *  the SDK's `motor_drive_master.c:cycle_step` design — every
     *  registered slot always gets an RPDO, regardless of CiA-402
     *  state. The cached controlword determines what the slave sees;
     *  unarmed slots stream @c 0x0000 which is a no-op for the FSM
     *  but keeps the slave's frame-driven CAN main loop alive so its
     *  SDO server stays responsive between bringup attempts.
     */
    void cycleStep();

    /** Number of slaves currently registered (for MasterWorker
     *  iteration). */
    uint32_t slaveCount() const { return static_cast<uint32_t>(m_slots.size()); }

    /** One OD entry slot in a PDO mapping (CiA 301 §7.2.2.2). */
    struct PdoMapEntry {
        uint16_t idx  = 0;
        uint8_t  sub  = 0;
        uint8_t  bits = 0;
    };

    /**
     * @brief Push a new PDO mapping to a slave via SDO.
     *
     * Follows the CiA 301 re-map dance:
     *   1. Disable the PDO (bit 31 of COB-ID at 0x1400+n / 0x1800+n sub 1).
     *   2. Clear the mapping count (0x1600+n / 0x1A00+n sub 0 = 0).
     *   3. Write each entry as @c (idx<<16)|(sub<<8)|bits into subs 1..N.
     *   4. Set the new mapping count at sub 0.
     *   5. Re-enable the PDO (clear bit 31 of COB-ID).
     *
     * Runs in the caller thread and blocks on the underlying USDO client.
     * Against a slave that doesn't expose the PDO param objects (for
     * example the in-tree motor_drive_cia402 simulator) every write aborts
     * with SDO code 0x06020000 and the function returns the first error.
     *
     * Only touches the first PDO slot (TPDO1 / RPDO1 — @c n == 0) which is
     * what the diagnostic configures. Extending this to higher slots is a
     * matter of adding a @c pdoSlot parameter.
     */
    int writePdoMapping(int slotIdx, bool isTpdo,
                        const std::vector<PdoMapEntry>& entries,
                        QString* err);

    /**
     * @brief Blocking SDO upload from a slave.
     *
     * Thin wrapper over @c co_fd_usdo_sync_read that looks up the slave's
     * pre-wired sync context so the caller just passes OD coordinates.
     * Runs on the caller thread; drives transport pumping while it waits.
     */
    int readSdo (int slotIdx, uint16_t odIdx, uint8_t odSub,
                 void* buf, uint32_t len, QString* err = nullptr);

    /** Blocking SDO download. Companion to @ref readSdo. */
    int writeSdo(int slotIdx, uint16_t odIdx, uint8_t odSub,
                 const void* buf, uint32_t len, QString* err = nullptr);

private:
    struct Slot;    /* defined in the .cpp */

    /* Two hal_can_t endpoints — one for SDO/USDO (single-consumer
     * rx_cb), one for PDO. UDP gives us cheap multi-socket fan-out;
     * ZLG can only do one consumer per channel today, so PDO is wired
     * for UDP only and left null on ZLG (no regression). */
    CanKind                        m_kind   = CanKind::Udp;
    hal_can_t*                     m_canCli = nullptr;
    hal_can_t*                     m_canPdo = nullptr;
    co_fd_usdo_client_t*           m_client = nullptr;
    can_fd_pdo_t*                  m_pdo    = nullptr;

    std::vector<Slot*>             m_slots;
};

}  // namespace vrmc
