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

/* UART + serial-bus drive transports. Gated at compile time so the
 * diagnostic still builds if the SDK was trimmed. */
#include "feetech/feetech_servo.h"
#include "hal_uart.h"
}

/* Dynamixel Protocol 2.0 driver lives in-tree (DynamixelDriver.cpp) so
 * the diagnostic always has the option in the Connect dialog, with no
 * rebuild or external SDK required. */
#include "DynamixelDriver.hpp"

namespace vrmc {

/**
 * @brief Which master-side transport to instantiate.
 *
 * The first two kinds are CAN-based CiA 402 drives; the last two are
 * RS-485 servo buses that share the same master_mgr abstraction via
 * the motor_drive_*_intf adapters in vr-mc-sdk.
 */
enum class CanKind {
    Udp       = 0,   /**< UDP-multicast "CAN" (loopback sim).            */
    Zlg       = 1,   /**< ZLG USB-CANFD adapter.                         */
    Feetech   = 2,   /**< Feetech SMS/STS RS-485 bus.                    */
    Dynamixel = 3    /**< ROBOTIS Dynamixel RS-485 bus.                  */
};

struct CanConfig
{
    /* --- Transport selector --- */
    CanKind  kind     = CanKind::Udp;

    /* --- UDP transport (hal_can_udp) --- */
    QString  group    = "239.192.0.42"; /**< UDP multicast group (matches drive). */
    uint16_t port     = 23400;          /**< UDP port.                    */

    /* --- ZLG USB-CANFD transport (hal_can_zlg) --- */
    QString  zlgLibPath  = "libcontrolcanfd.so"; /**< dlopen() target.   */
    uint32_t zlgChannel  = 0;                    /**< channel index 0/1. */
    uint32_t zlgBitrate  = 1'000'000;            /**< arb-phase bps.     */
    uint32_t zlgFdBitrate = 4'000'000;           /**< data-phase bps.    */

    /* --- UART-based transports (Feetech / Dynamixel) --- */
    QString  uartDevice  = "/dev/ttyUSB0";       /**< Serial port path.  */
    uint32_t uartBaud    = 1'000'000;            /**< Feetech default;   */
                                                  /**< Dynamixel common. */
    uint8_t  dxlProtocolVer = 2;                 /**< 1 or 2.            */

    /* --- Common --- */
    uint8_t  first_id       = 5;        /**< First slave id.              */
    uint8_t  count          = 1;        /**< Number of slaves to bring up.*/
    uint32_t sdo_timeout_ms = 100;
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

    /** Latest TPDO snapshot from a slave, raw drive units. */
    struct PdoFrame {
        uint16_t statusword = 0;
        int32_t  position   = 0;     /**< raw counts                     */
        int32_t  velocity   = 0;     /**< raw counts/s                   */
        int16_t  torque     = 0;     /**< per-mille of rated torque      */
        uint16_t error_code = 0;
        uint64_t rx_count   = 0;     /**< running PDO RX counter         */
        bool     fresh      = false; /**< true if any PDO yet seen       */
    };

    /** Atomically copy out a slave's most recent PDO snapshot. */
    bool getPdo(uint32_t idx, PdoFrame* out) const;

    /** @return true when the PDO layer is wired up for the current
     *  transport (UDP). Worker uses this to decide whether telemetry
     *  should come from cached TPDO frames (fast, passive) or from
     *  per-tick SDO reads (slow, blast-y). */
    bool hasPdo() const { return m_pdo != nullptr; }

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

    /* Serial bus transports. These don't use hal_can at all — they're
     * UART-based RS-485 servo chains. Only populated when @c m_kind is
     * Feetech or Dynamixel. */
    hal_uart_t*                    m_uart        = nullptr;
    feetech_servo_t                m_feetechBus  = {};
    DynamixelBus*                  m_dxlBus      = nullptr;

    std::vector<Slot*>             m_slots;
};

}  // namespace vrmc
