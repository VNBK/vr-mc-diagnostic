/**
 * @file   BwFromKp.hpp
 * @brief  Back-compute the equivalent closed-loop bandwidth (Hz) from a PI
 *         proportional gain plus motor name-plate, by inverting the same
 *         formulas the board-side @c motor_tune_*_pi routines apply.
 *
 *  Used by the diagnostic to display "what BW does the live Kp correspond to"
 *  whenever the worker reads back 0x60F6 / 0x60F9 / 0x60FB. The board does
 *  not persist BW per loop, so Kp + the cached motor profile is the only
 *  consistent source of truth (and it stays right even if the operator
 *  edited Kp manually outside the tune path).
 */

#pragma once

#include "MasterWorker.hpp"     // vrmc::Loop
#include "MotorProfile.hpp"     // vrmc::MotorParams, MotorType

#include <cmath>

namespace vrmc {

/** @return BW in Hz that, fed into the auto-tune, would produce the same
 *          Kp on the given loop. NaN if the required profile field is
 *          zero / missing (callers should leave the spinbox untouched in
 *          that case). */
inline float bwFromKp(Loop loop, float kp, const MotorParams& p)
{
    constexpr float kTwoPi = 6.2831853071795864769f;
    /* Damping ratio that motor_tune_velocity_pi / motor_tune_position_pi
     * bake into Kp (both target Butterworth 2nd-order). Must match the
     * board's hard-coded value (motor_drive.c:1504 / 1536) -- a mismatch
     * here surfaces as the displayed BW being off by 1/(2*zeta) = sqrt(2)
     * for the same gains. */
    constexpr float kZeta  = 0.707f;

    switch (loop){
    case Loop::Current: {
        /* motor_tune_current_pi: Kp = Ls_q * ωc  =>  BW = Kp / (2π * Ls_q).
         * No damping factor here -- the current loop is tuned by pole
         * cancellation, not 2nd-order placement. */
        if (p.ls_q <= 0.0f){ return NAN; }
        return kp / (kTwoPi * p.ls_q);
    }
    case Loop::Velocity: {
        /* motor_tune_velocity_pi: Kp = 2·ζ·ωc · J/Kt
         *                      =>  BW = Kp · Kt / (2·ζ · 2π · J)
         * Kt comes from 0x2070:9 when populated; otherwise derive 1.5·N·λ
         * for PMSM or fall back to torque_constant for BLDC. */
        if (p.inertia <= 0.0f){ return NAN; }
        float kt = p.torque_constant;
        if (kt <= 0.0f){
            if (p.type == MotorType::Pmsm){
                kt = 1.5f * static_cast<float>(p.pole_pair) * p.rated_flux;
            } else {
                /* No usable Kt -> can't back-compute. */
                return NAN;
            }
        }
        if (kt <= 0.0f){ return NAN; }
        return kp * kt / (2.0f * kZeta * kTwoPi * p.inertia);
    }
    case Loop::Position: {
        /* motor_tune_position_pi: Kp = 2·ζ·ωc (plant is 1/s integrator)
         *                      =>  BW = Kp / (2·ζ · 2π) */
        return kp / (2.0f * kZeta * kTwoPi);
    }
    }
    return NAN;
}

}  // namespace vrmc
