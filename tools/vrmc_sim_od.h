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
 *   - @c 0x6076 Motor rated torque            U32  RW  μNm
 *   - @c 0x6078 Current actual value          I16  RO  mA
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
 */
#define CIA402_DRIVE_OD_COUNT       (6u)

/**
 * @brief Default rated torque in micro-Newton-metres (= 0.5 Nm).
 *
 * Mirrors the @c RATED_TORQUE_NM compile-time constant baked into the
 * apps' Nm <-> per-mille conversion. Keep in sync if you change the
 * scaling there.
 */
#define CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_uNm  (500000u)

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

    /** @brief @c 0x6076 — motor rated torque (μNm). */
    uint32_t  motor_rated_torque_uNm;

    /** @brief @c 0x6078 — current actual value (mA, signed). */
    int16_t   current_actual_mA;

    /** @brief @c 0x607C — home offset (position increments, signed). */
    int32_t   home_offset;

    /** @brief @c 0x607D.1 — software min position limit (increments). */
    int32_t   pos_limit_min;

    /** @brief @c 0x607D.2 — software max position limit (increments). */
    int32_t   pos_limit_max;

    /** @brief @c 0x6080 — max motor speed (rpm; or app-defined units). */
    uint32_t  max_motor_speed;

    /* ---------------- internal wiring (do not touch) ---------------- */

    uint8_t      n_607D_subs;           /* const 2; backs 0x607D.0      */
    co_sub_obj_t sub_6072[1];
    co_sub_obj_t sub_6076[1];
    co_sub_obj_t sub_6078[1];
    co_sub_obj_t sub_607C[1];
    co_sub_obj_t sub_607D[3];           /* 0=count, 1=min, 2=max        */
    co_sub_obj_t sub_6080[1];
    co_obj_t     objs[CIA402_DRIVE_OD_COUNT];
} cia402_drive_od_t;

/**
 * @brief Populate @p _x with sane defaults and wire its sub-tables.
 *
 * Defaults:
 * - @c max_torque                 = 1000  (100 % of rated)
 * - @c motor_rated_torque_uNm     = @ref CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_uNm
 * - @c current_actual_mA          = 0
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
