/**
 * @file   hal_can_zlg.hpp
 * @brief  ZLG USB-CANFD adapter implementing the vr-mc-sdk @c hal_can_t
 *         vtable, mirroring how @c hal_can_udp exposes a CAN transport.
 *
 * Construction opens a ZLG device + channel through the in-tree
 * @ref can_drv::ZlgCanDrv class (loads @c libcontrolcanfd.so via dlopen),
 * configures it for CAN-FD at the requested bitrate, and starts a
 * background poll thread that drains @c readAsync into the registered
 * @ref hal_can_rx_fn_t callback. The returned @ref hal_can_t* is shape-
 * compatible with everything @c can_stack expects (USDO client, PDO
 * engine, etc.), so the existing @c CanBackend can swap UDP for ZLG by
 * swapping this constructor.
 *
 * Lifetime: caller owns the returned pointer and destroys it via
 * @ref hal_can_zlg_destroy. Internally we keep the ZlgCanDrv shared_ptr
 * alive for as long as the channel is in use.
 */

#pragma once

extern "C" {
#include "hal_can.h"
}

#include <cstdint>
#include <string>

namespace vrmc {

/**
 * @brief Build a ZLG CAN transport.
 *
 * @param _lib_path     Filename or full path to @c libcontrolcanfd.so.
 *                      Pass "libcontrolcanfd.so" to rely on the loader's
 *                      RPATH (CMake sets one for us).
 * @param _channel      ZLG channel index (0 or 1 on a USB-CANFD-200U).
 * @param _bitrate_bps  Arbitration-phase bitrate (e.g. 1'000'000).
 * @param _fd_bps       Data-phase bitrate (e.g. 4'000'000). Pass 0 to
 *                      disable BRS / match arbitration rate.
 * @return Owning @ref hal_can_t* on success, @c nullptr if the lib or
 *         device could not be opened.
 */
hal_can_t* hal_can_zlg_create(const std::string& _lib_path,
                              uint32_t            _channel,
                              uint32_t            _bitrate_bps,
                              uint32_t            _fd_bps);

/** Tear down a transport built by @ref hal_can_zlg_create. */
void hal_can_zlg_destroy(hal_can_t* _this);

}  // namespace vrmc
