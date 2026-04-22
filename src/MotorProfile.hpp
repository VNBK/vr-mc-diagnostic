/**
 * @file   MotorProfile.hpp
 * @brief  Motor name-plate + electrical parameters used by the FOC tuner,
 *         plus connection settings for round-tripping a session.
 *
 * Field set mirrors @c motor_profile_t in
 * @c vr-mc-sdk/src/mc/motor_profile.h. Units stay SI (Ω, H, Wb, kg·m²,
 * rad/s, V, A) so they can be pushed straight into the SDK structs once
 * the diagnostic exposes a "write to drive" path.
 *
 * JSON schema (version 1):
 * @code
 *  {
 *    "version": 1,
 *    "motor": {
 *      "type":           "PMSM",   // or "BLDC"
 *      "pole_pair":      4,
 *      "rs":             0.5,      // Ω
 *      "ls_d":           0.0015,   // H
 *      "ls_q":           0.0015,   // H
 *      "rated_flux":     0.05,     // Wb
 *      "inertia":        5.0e-5,   // kg·m²
 *      "rated_speed":    1000,     // rad/s (or rpm — see SDK)
 *      "rated_vol":      24,       // V
 *      "min_vol":        12,       // V
 *      "max_vol":        36,       // V
 *      "rated_cur":      2,        // A
 *      "max_cur":        6,        // A
 *      "stall_cur":      8,        // A
 *      "stall_time_cur": 1         // s
 *    },
 *    "connection": { ... }         // optional, same shape as before
 *  }
 * @endcode
 */

#pragma once

#include "backends/CanBackend.hpp"
#include <QString>
#include <cstdint>

namespace vrmc {

/** @brief Motor topology — mirrors @c MOTOR_TYPE_BLDC / PMSM. */
enum class MotorType { Bldc = 0, Pmsm = 1 };

/** @brief Name-plate + electrical params (1:1 with @c motor_profile_t). */
struct MotorParams
{
    MotorType type           = MotorType::Pmsm;
    uint32_t  pole_pair      = 4;
    float     rs             = 0.5f;     /**< stator phase resistance, Ω    */
    float     ls_d           = 0.0015f;  /**< d-axis inductance, H          */
    float     ls_q           = 0.0015f;  /**< q-axis inductance, H          */
    float     rated_flux     = 0.05f;    /**< rotor PM flux λ_m, Wb         */
    float     inertia        = 5.0e-5f;  /**< rotor inertia J, kg·m²        */
    int       rated_speed    = 1000;     /**< name-plate rated speed        */
    int       rated_vol      = 24;       /**< rated DC-bus voltage, V       */
    int       min_vol        = 12;       /**< undervoltage trip, V          */
    int       max_vol        = 36;       /**< overvoltage trip, V           */
    int       rated_cur      = 2;        /**< rated phase current, A        */
    int       max_cur        = 6;        /**< peak allowable phase cur, A   */
    int       stall_cur      = 8;        /**< current considered "stall", A */
    int       stall_time_cur = 1;        /**< time at stall_cur before trip */
};

struct MotorProfile
{
    int          schemaVersion = 1;
    MotorParams  motor;
    CanConfig    connection;
};

namespace profile {

/** Serialise to a JSON file. Returns true on success. */
bool save(const QString& path, const MotorProfile& mp, QString* err = nullptr);

/** Read a JSON file. Returns true + fills @p mp_out on success. */
bool load(const QString& path, MotorProfile* mp_out, QString* err = nullptr);

/** Helper: motor topology <-> JSON-friendly string. */
QString   motorTypeToString  (MotorType t);
MotorType motorTypeFromString(const QString& s);

}  // namespace profile

}  // namespace vrmc
