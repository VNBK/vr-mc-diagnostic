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
 * JSON schema (version 2):
 * @code
 *  {
 *    "version": 2,
 *    "motor": {
 *      "type":            "PMSM",   // or "BLDC"
 *      "pole_pair":       4,
 *      "rs":              0.5,      // Ω
 *      "ls_d":            0.0015,   // H
 *      "ls_q":            0.0015,   // H
 *      "rated_flux":      0.05,     // Wb
 *      "inertia":         5.0e-5,   // kg·m²
 *      "rated_torque":    0.5,      // Nm  (0x6076)
 *      "rated_speed":     1000,     // rad/s (or rpm — see SDK)
 *      "rated_vol":       24,       // V
 *      "rated_cur":       2,        // A
 *      "enc_increments":  16384,    // 0x608F:1
 *      "enc_motor_revs":  1         // 0x608F:2  (counts/rev = inc / revs)
 *    },
 *    "connection": { ... }          // optional, same shape as before
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

/** @brief Name-plate + electrical params. */
struct MotorParams
{
    MotorType type           = MotorType::Pmsm;
    uint32_t  pole_pair      = 4;
    float     rs             = 0.5f;     /**< stator phase resistance, Ω    */
    float     ls_d           = 0.0015f;  /**< d-axis inductance, H          */
    float     ls_q           = 0.0015f;  /**< q-axis inductance, H          */
    float     rated_flux     = 0.05f;    /**< rotor PM flux λ_m, Wb         */
    float     inertia        = 5.0e-5f;  /**< rotor inertia J, kg·m²        */
    float     rated_torque   = 0.5f;     /**< rated torque, Nm (0x6076)     */
    float     torque_constant = 0.0f;    /**< Kt, Nm/A (0x2070:9). 0 = drive
                                          *   derives 1.5·pole·flux (PMSM).  */
    int       rated_speed    = 1000;     /**< name-plate rated speed        */
    int       rated_vol      = 24;       /**< rated DC-bus voltage, V       */
    float     rated_cur      = 2.0f;     /**< rated phase current, A (float
                                          *   for sub-amp motors). 0x6075 mA */

    /* Encoder resolution as a CiA-402 0x608F ratio (like a gearbox):
     * counts_per_rev = enc_increments / enc_motor_revs. */
    uint32_t  enc_increments = 16384;    /**< 0x608F:1 encoder increments   */
    uint32_t  enc_motor_revs = 1;        /**< 0x608F:2 motor revolutions    */

    /* Incremental-encoder quadrature counts/rev (= 4 × lines), 0x2070:11.
     * Only present on the encoder drive variant (3FL_2). */
    uint32_t  cpr            = 4000;     /**< 0x2070:11 encoder CPR         */
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
