/**
 * @file   DynamixelDriver.hpp
 * @brief  Self-contained ROBOTIS Dynamixel Protocol 2.0 driver.
 *
 * The diagnostic used to gate Dynamixel behind the ROBOTIS DYNAMIXEL
 * SDK (`port_handler.h`, `packet_handler.h`), which meant anyone who
 * wanted it had to rebuild with `-DVRMC_BUILD_DYNAMIXEL=ON` plus point
 * at an external checkout. That's a nuisance for ad-hoc bench work.
 *
 * This module reimplements just enough of Protocol 2.0 — packet
 * framing, CRC-16/IBM, byte stuffing, Ping / Read / Write
 * instructions — to talk to the Dynamixel-X control table (XL / XM / XH
 * series). It is always compiled into the diagnostic, so picking
 * "Dynamixel" in the Connect dialog Just Works without external
 * dependencies or a rebuild.
 *
 * Two types:
 *   - @ref DynamixelBus owns an open serial port; many adapters share it.
 *   - @ref makeDynamixelIntf builds a @c motor_drive_intf_t* bound to
 *     one servo id on that bus. The returned pointer plugs straight into
 *     @c master_mgr_add_slave.
 */

#pragma once

#include <cstdint>
#include <string>

extern "C" {
#include "motor_drive_interface.h"
}

namespace vrmc {

class DynamixelBus
{
public:
    /** Open @p device at @p baud (standard Dynamixel values: 57600,
     *  115200, 1Mbps). Returns @c nullptr on failure; caller destroys
     *  with `delete`. */
    static DynamixelBus* open(const std::string& device, uint32_t baud,
                              std::string* err = nullptr);
    ~DynamixelBus();

    DynamixelBus(const DynamixelBus&)            = delete;
    DynamixelBus& operator=(const DynamixelBus&) = delete;

    /** True when the bus was brought up successfully. */
    bool ok() const { return m_fd >= 0; }

    /** Protocol 2.0 ping. Writes a ping packet and waits for the status
     *  reply. @return 0 on success. */
    int ping(uint8_t id);

    /** Read @p len bytes from control-table @p addr. @p buf must be at
     *  least @p len bytes long. */
    int read (uint8_t id, uint16_t addr, uint16_t len, uint8_t* buf);

    /** Write @p len bytes of @p buf into control-table @p addr. */
    int write(uint8_t id, uint16_t addr, uint16_t len, const uint8_t* buf);

private:
    DynamixelBus() = default;

    int   m_fd      = -1;
    /* One outstanding transaction at a time — shared-bus RS-485. */
    uint8_t m_rxbuf[256] = {};
};

/** Extra knobs for the per-servo adapter. */
struct DynamixelIntfCfg {
    DynamixelBus* bus   = nullptr;
    uint8_t       id    = 1;
    const char*   name  = nullptr;     /**< NULL → "dxl-<id>".        */
    /** Torque (Nm) → goal-current (mA) scale. 0 treats set_torque as
     *  raw current in A. */
    float         A_per_Nm = 0.0f;
    /** Default operating mode applied by bringup() before torque on.
     *  Most benches use POSITION (3) or VELOCITY (1). */
    motor_intf_mode_t bringup_mode = MOTOR_INTF_MODE_POSITION;
};

/** Build a @c motor_drive_intf_t* bound to one Dynamixel id. Returns
 *  nullptr on bad args. Destroy via @c motor_drive_intf_free. */
motor_drive_intf_t* makeDynamixelIntf(const DynamixelIntfCfg& cfg);

}  // namespace vrmc
