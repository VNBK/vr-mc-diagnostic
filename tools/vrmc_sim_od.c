/*
 * cia402_drive_od.c — shared OD-extension block for the CiA 402 drive
 * apps. See cia402_drive_od.h for the index list and rationale.
 */

#include "vrmc_sim_od.h"

#include <limits.h>
#include <string.h>


static void install_sub(co_sub_obj_t* s, void* data, uint16_t len,
                        uint8_t data_type, uint8_t access){
    memset(s, 0, sizeof(*s));
    s->data           = data;
    s->len            = len;
    s->data_type      = data_type;
    s->access         = access;
    s->data_access_fn = NULL;   /* no async write notification needed   */
    s->arg            = NULL;
}


void cia402_drive_od_reset(cia402_drive_od_t* _x){
    _x->max_torque             = 1000u;        /* 100 % rated         */
    _x->motor_rated_torque_mNm = CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_mNm;
    _x->current_actual_permille = 0;
    _x->home_offset            = 0;
    _x->pos_limit_min          = INT32_MIN;
    _x->pos_limit_max          = INT32_MAX;
    _x->max_motor_speed        = UINT32_MAX;   /* no clamp by default */
    _x->n_607D_subs            = 2u;

    /* Identity (mirrors the board's sim variant). */
    _x->device_type  = 0x00020192u;            /* servo, CiA-402      */
    _x->vendor_id    = 0x000004D1u;            /* placeholder (DUALI) */
    _x->product_code = 0x0001E000u;            /* drive simulator     */
    _x->revision     = 0x00010000u;            /* 1.0                 */
    _x->serial       = 0u;
    strncpy(_x->dev_name, "vrmc-sim", sizeof(_x->dev_name) - 1);
    strncpy(_x->hw_ver,   "sim",      sizeof(_x->hw_ver)   - 1);
    strncpy(_x->sw_ver,   "1.0.0",    sizeof(_x->sw_ver)   - 1);

    /* Protection / rating / profile / homing. */
    _x->rated_current_mA      = 2000u;         /* 2 A                 */
    _x->following_err_window  = 4096u;
    _x->following_err_timeout = 100u;
    _x->profile_velocity      = 16384u;        /* 1 rev/s             */
    _x->profile_accel         = 81920u;
    _x->profile_decel         = 81920u;
    _x->quickstop_decel       = 163840u;
    _x->homing_method         = 0;
    _x->homing_speed_switch   = 8192u;
    _x->homing_speed_zero     = 819u;
    _x->homing_accel          = 16384u;

    /* Standard control gains (plausible non-zero so Read shows values). */
    _x->cur_kp = 2.5f;  _x->cur_ki = 120.0f;
    _x->vel_kp = 0.08f; _x->vel_ki = 4.0f;
    _x->pos_kp = 30.0f; _x->pos_ki = 0.0f;

    /* Encoder resolution: 14-bit, 16384 counts / 1 motor rev. */
    _x->enc_increments = 16384u;
    _x->enc_motor_revs = 1u;

    /* Manufacturer calibration / profile (plausible non-zero defaults). */
    _x->curr_off_a  = 0.0f;  _x->curr_off_b  = 0.0f;  _x->curr_off_c  = 0.0f;
    _x->curr_gain_a = 1.0f;  _x->curr_gain_b = 1.0f;  _x->curr_gain_c = 1.0f;
    _x->stall_current_mA = 6000u;
    _x->stall_time_ms    = 500u;
    _x->hall_offset = 0.0f;
    _x->prof_type      = 1u;       /* PMSM */
    _x->prof_pole_pair = 4u;
    _x->prof_rs        = 0.5f;
    _x->prof_ls_d      = 0.0015f;
    _x->prof_ls_q      = 0.0015f;
    _x->prof_flux      = 0.05f;
    _x->prof_inertia   = 5.0e-5f;
    _x->prof_rated_vol = 24;

    _x->n_1018_subs = 4u;
    _x->n_6099_subs = 2u;
    _x->n_gain_subs = 2u;
    _x->n_608F_subs = 2u;
    _x->n_2040_subs = 3u;
    _x->n_2041_subs = 3u;
    _x->n_2050_subs = 2u;
    _x->n_2070_subs = 8u;
    _x->n_2031_subs = 2u;
    _x->vf_freq_mhz   = 0;
    _x->vf_amp_milli = 0u;
    _x->n_2032_subs = 3u;
    _x->vf_bsp_enable    = 0u;
    _x->vf_bsp_freq_mhz  = 0;
    _x->vf_bsp_amp_milli = 0u;
}


void cia402_drive_od_init(cia402_drive_od_t* _x){
    if(!_x){ return; }
    memset(_x, 0, sizeof(*_x));
    cia402_drive_od_reset(_x);

    /* 0x6072 Max torque (RW U16) */
    install_sub(&_x->sub_6072[0], &_x->max_torque,
                sizeof(_x->max_torque), CO_DT_UINT16, CO_ACCESS_RW);
    _x->objs[0] = (co_obj_t){ .index = 0x6072u, .sub_number = 1,
                               .subs = _x->sub_6072 };

    /* 0x6076 Motor rated torque (RO U32, mNm) -- DERIVED = Kt * rated
     * current (recomputed each tick in vrmc_sim.c), mirroring the drive. */
    install_sub(&_x->sub_6076[0], &_x->motor_rated_torque_mNm,
                sizeof(_x->motor_rated_torque_mNm), CO_DT_UINT32,
                CO_ACCESS_RO);
    _x->objs[1] = (co_obj_t){ .index = 0x6076u, .sub_number = 1,
                               .subs = _x->sub_6076 };

    /* 0x6078 Current actual value (RO I16, per-mille of rated, PDO-TX) */
    install_sub(&_x->sub_6078[0], &_x->current_actual_permille,
                sizeof(_x->current_actual_permille), CO_DT_INT16,
                CO_ACCESS_RO | CO_ACCESS_PDO_MAPPABLE | CO_ACCESS_PDO_TX_ONLY);
    _x->objs[2] = (co_obj_t){ .index = 0x6078u, .sub_number = 1,
                               .subs = _x->sub_6078 };

    /* 0x607C Home offset (RW I32) */
    install_sub(&_x->sub_607C[0], &_x->home_offset,
                sizeof(_x->home_offset), CO_DT_INT32, CO_ACCESS_RW);
    _x->objs[3] = (co_obj_t){ .index = 0x607Cu, .sub_number = 1,
                               .subs = _x->sub_607C };

    /* 0x607D Software position limit (record):
     *   .0  number of entries  (RO U8, const 2)
     *   .1  min position limit (RW I32)
     *   .2  max position limit (RW I32)
     */
    install_sub(&_x->sub_607D[0], &_x->n_607D_subs,
                sizeof(_x->n_607D_subs), CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_607D[1], &_x->pos_limit_min,
                sizeof(_x->pos_limit_min), CO_DT_INT32, CO_ACCESS_RW);
    install_sub(&_x->sub_607D[2], &_x->pos_limit_max,
                sizeof(_x->pos_limit_max), CO_DT_INT32, CO_ACCESS_RW);
    _x->objs[4] = (co_obj_t){ .index = 0x607Du, .sub_number = 3,
                               .subs = _x->sub_607D };

    /* 0x6080 Max motor speed (RW U32, rpm) */
    install_sub(&_x->sub_6080[0], &_x->max_motor_speed,
                sizeof(_x->max_motor_speed), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[5] = (co_obj_t){ .index = 0x6080u, .sub_number = 1,
                               .subs = _x->sub_6080 };

    /* ---- Identity (0x1000 / 0x1008 / 0x1009 / 0x100A / 0x1018) ---- */
    install_sub(&_x->sub_1000[0], &_x->device_type,
                sizeof(_x->device_type), CO_DT_UINT32, CO_ACCESS_CONST);
    _x->objs[6] = (co_obj_t){ .index = 0x1000u, .sub_number = 1,
                               .subs = _x->sub_1000 };

    install_sub(&_x->sub_1008[0], _x->dev_name,
                (uint16_t)strlen(_x->dev_name), CO_DT_VISIBLE_STR, CO_ACCESS_CONST);
    _x->objs[7] = (co_obj_t){ .index = 0x1008u, .sub_number = 1,
                               .subs = _x->sub_1008 };

    install_sub(&_x->sub_1009[0], _x->hw_ver,
                (uint16_t)strlen(_x->hw_ver), CO_DT_VISIBLE_STR, CO_ACCESS_CONST);
    _x->objs[8] = (co_obj_t){ .index = 0x1009u, .sub_number = 1,
                               .subs = _x->sub_1009 };

    install_sub(&_x->sub_100A[0], _x->sw_ver,
                (uint16_t)strlen(_x->sw_ver), CO_DT_VISIBLE_STR, CO_ACCESS_CONST);
    _x->objs[9] = (co_obj_t){ .index = 0x100Au, .sub_number = 1,
                               .subs = _x->sub_100A };

    install_sub(&_x->sub_1018[0], &_x->n_1018_subs,
                sizeof(_x->n_1018_subs), CO_DT_UINT8,  CO_ACCESS_CONST);
    install_sub(&_x->sub_1018[1], &_x->vendor_id,
                sizeof(_x->vendor_id),   CO_DT_UINT32, CO_ACCESS_CONST);
    install_sub(&_x->sub_1018[2], &_x->product_code,
                sizeof(_x->product_code), CO_DT_UINT32, CO_ACCESS_CONST);
    install_sub(&_x->sub_1018[3], &_x->revision,
                sizeof(_x->revision),    CO_DT_UINT32, CO_ACCESS_CONST);
    install_sub(&_x->sub_1018[4], &_x->serial,
                sizeof(_x->serial),      CO_DT_UINT32, CO_ACCESS_CONST);
    _x->objs[10] = (co_obj_t){ .index = 0x1018u, .sub_number = 5,
                                .subs = _x->sub_1018 };

    /* ---- Protection (0x6065 / 0x6066) + rated current (0x6075) ---- */
    install_sub(&_x->sub_6065[0], &_x->following_err_window,
                sizeof(_x->following_err_window), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[11] = (co_obj_t){ .index = 0x6065u, .sub_number = 1,
                                .subs = _x->sub_6065 };

    install_sub(&_x->sub_6066[0], &_x->following_err_timeout,
                sizeof(_x->following_err_timeout), CO_DT_UINT16, CO_ACCESS_RW);
    _x->objs[12] = (co_obj_t){ .index = 0x6066u, .sub_number = 1,
                                .subs = _x->sub_6066 };

    install_sub(&_x->sub_6075[0], &_x->rated_current_mA,
                sizeof(_x->rated_current_mA), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[13] = (co_obj_t){ .index = 0x6075u, .sub_number = 1,
                                .subs = _x->sub_6075 };

    /* ---- Motion profile (0x6081 / 0x6083 / 0x6084 / 0x6085) ---- */
    install_sub(&_x->sub_6081[0], &_x->profile_velocity,
                sizeof(_x->profile_velocity), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[14] = (co_obj_t){ .index = 0x6081u, .sub_number = 1,
                                .subs = _x->sub_6081 };
    install_sub(&_x->sub_6083[0], &_x->profile_accel,
                sizeof(_x->profile_accel), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[15] = (co_obj_t){ .index = 0x6083u, .sub_number = 1,
                                .subs = _x->sub_6083 };
    install_sub(&_x->sub_6084[0], &_x->profile_decel,
                sizeof(_x->profile_decel), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[16] = (co_obj_t){ .index = 0x6084u, .sub_number = 1,
                                .subs = _x->sub_6084 };
    install_sub(&_x->sub_6085[0], &_x->quickstop_decel,
                sizeof(_x->quickstop_decel), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[17] = (co_obj_t){ .index = 0x6085u, .sub_number = 1,
                                .subs = _x->sub_6085 };

    /* ---- Homing (0x6098 / 0x6099 / 0x609A) ---- */
    install_sub(&_x->sub_6098[0], &_x->homing_method,
                sizeof(_x->homing_method), CO_DT_INT8, CO_ACCESS_RW);
    _x->objs[18] = (co_obj_t){ .index = 0x6098u, .sub_number = 1,
                                .subs = _x->sub_6098 };

    install_sub(&_x->sub_6099[0], &_x->n_6099_subs,
                sizeof(_x->n_6099_subs), CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_6099[1], &_x->homing_speed_switch,
                sizeof(_x->homing_speed_switch), CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_6099[2], &_x->homing_speed_zero,
                sizeof(_x->homing_speed_zero), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[19] = (co_obj_t){ .index = 0x6099u, .sub_number = 3,
                                .subs = _x->sub_6099 };

    install_sub(&_x->sub_609A[0], &_x->homing_accel,
                sizeof(_x->homing_accel), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[20] = (co_obj_t){ .index = 0x609Au, .sub_number = 1,
                                .subs = _x->sub_609A };

    /* ---- Standard control gains (0x60F6 / 0x60F9 / 0x60FB) ---- */
    install_sub(&_x->sub_60F6[0], &_x->n_gain_subs,
                sizeof(_x->n_gain_subs), CO_DT_UINT8,   CO_ACCESS_CONST);
    install_sub(&_x->sub_60F6[1], &_x->cur_kp,
                sizeof(_x->cur_kp), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_60F6[2], &_x->cur_ki,
                sizeof(_x->cur_ki), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[21] = (co_obj_t){ .index = 0x60F6u, .sub_number = 3,
                                .subs = _x->sub_60F6 };

    install_sub(&_x->sub_60F9[0], &_x->n_gain_subs,
                sizeof(_x->n_gain_subs), CO_DT_UINT8,   CO_ACCESS_CONST);
    install_sub(&_x->sub_60F9[1], &_x->vel_kp,
                sizeof(_x->vel_kp), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_60F9[2], &_x->vel_ki,
                sizeof(_x->vel_ki), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[22] = (co_obj_t){ .index = 0x60F9u, .sub_number = 3,
                                .subs = _x->sub_60F9 };

    install_sub(&_x->sub_60FB[0], &_x->n_gain_subs,
                sizeof(_x->n_gain_subs), CO_DT_UINT8,   CO_ACCESS_CONST);
    install_sub(&_x->sub_60FB[1], &_x->pos_kp,
                sizeof(_x->pos_kp), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_60FB[2], &_x->pos_ki,
                sizeof(_x->pos_ki), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[23] = (co_obj_t){ .index = 0x60FBu, .sub_number = 3,
                                .subs = _x->sub_60FB };

    /* ---- Position encoder resolution (0x608F, like a gearbox) ---- */
    install_sub(&_x->sub_608F[0], &_x->n_608F_subs,
                sizeof(_x->n_608F_subs), CO_DT_UINT8,  CO_ACCESS_CONST);
    install_sub(&_x->sub_608F[1], &_x->enc_increments,
                sizeof(_x->enc_increments), CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_608F[2], &_x->enc_motor_revs,
                sizeof(_x->enc_motor_revs), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[24] = (co_obj_t){ .index = 0x608Fu, .sub_number = 3,
                                .subs = _x->sub_608F };

    /* ---- 0x2040 current sensor offsets (RW) ---- */
    install_sub(&_x->sub_2040[0], &_x->n_2040_subs, sizeof(_x->n_2040_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2040[1], &_x->curr_off_a, sizeof(_x->curr_off_a), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2040[2], &_x->curr_off_b, sizeof(_x->curr_off_b), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2040[3], &_x->curr_off_c, sizeof(_x->curr_off_c), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[25] = (co_obj_t){ .index = 0x2040u, .sub_number = 4, .subs = _x->sub_2040 };

    /* ---- 0x2041 current sensor gains (RW) ---- */
    install_sub(&_x->sub_2041[0], &_x->n_2041_subs, sizeof(_x->n_2041_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2041[1], &_x->curr_gain_a, sizeof(_x->curr_gain_a), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2041[2], &_x->curr_gain_b, sizeof(_x->curr_gain_b), CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2041[3], &_x->curr_gain_c, sizeof(_x->curr_gain_c), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[26] = (co_obj_t){ .index = 0x2041u, .sub_number = 4, .subs = _x->sub_2041 };

    /* ---- 0x2050 stall protection (RW) ---- */
    install_sub(&_x->sub_2050[0], &_x->n_2050_subs, sizeof(_x->n_2050_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2050[1], &_x->stall_current_mA, sizeof(_x->stall_current_mA), CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_2050[2], &_x->stall_time_ms,    sizeof(_x->stall_time_ms),    CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[27] = (co_obj_t){ .index = 0x2050u, .sub_number = 3, .subs = _x->sub_2050 };

    /* ---- 0x2060 Hall offset (VAR, RW) ---- */
    install_sub(&_x->sub_2060[0], &_x->hall_offset, sizeof(_x->hall_offset), CO_DT_REAL32, CO_ACCESS_RW);
    _x->objs[28] = (co_obj_t){ .index = 0x2060u, .sub_number = 1, .subs = _x->sub_2060 };

    /* ---- 0x2070 motor-profile electrical record (RW) ---- */
    install_sub(&_x->sub_2070[0], &_x->n_2070_subs, sizeof(_x->n_2070_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2070[1], &_x->prof_type,      sizeof(_x->prof_type),      CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[2], &_x->prof_pole_pair, sizeof(_x->prof_pole_pair), CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[3], &_x->prof_rs,        sizeof(_x->prof_rs),        CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[4], &_x->prof_ls_d,      sizeof(_x->prof_ls_d),      CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[5], &_x->prof_ls_q,      sizeof(_x->prof_ls_q),      CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[6], &_x->prof_flux,      sizeof(_x->prof_flux),      CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[7], &_x->prof_inertia,   sizeof(_x->prof_inertia),   CO_DT_REAL32, CO_ACCESS_RW);
    install_sub(&_x->sub_2070[8], &_x->prof_rated_vol, sizeof(_x->prof_rated_vol), CO_DT_INT32,  CO_ACCESS_RW);
    _x->objs[29] = (co_obj_t){ .index = 0x2070u, .sub_number = 9, .subs = _x->sub_2070 };

    /* ---- 0x2031 open-loop V/f setpoint (RW) ---- *
     * Stored only (the host sim has no V/f motor model) so the diagnostic's
     * "Start V/f" SDO writes succeed over loopback instead of aborting. */
    install_sub(&_x->sub_2031[0], &_x->n_2031_subs, sizeof(_x->n_2031_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2031[1], &_x->vf_freq_mhz,   sizeof(_x->vf_freq_mhz),   CO_DT_INT32,  CO_ACCESS_RW);
    install_sub(&_x->sub_2031[2], &_x->vf_amp_milli, sizeof(_x->vf_amp_milli), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[30] = (co_obj_t){ .index = 0x2031u, .sub_number = 3, .subs = _x->sub_2031 };

    /* ---- 0x2032 BSP open-loop V/f (RW) ---- *
     * Stored only (host sim has no bsp_vf) so the diagnostic's BSP-engine
     * "Start V/f" SDO writes succeed over loopback. */
    install_sub(&_x->sub_2032[0], &_x->n_2032_subs, sizeof(_x->n_2032_subs),
                CO_DT_UINT8, CO_ACCESS_CONST);
    install_sub(&_x->sub_2032[1], &_x->vf_bsp_enable,    sizeof(_x->vf_bsp_enable),    CO_DT_UINT32, CO_ACCESS_RW);
    install_sub(&_x->sub_2032[2], &_x->vf_bsp_freq_mhz,  sizeof(_x->vf_bsp_freq_mhz),  CO_DT_INT32,  CO_ACCESS_RW);
    install_sub(&_x->sub_2032[3], &_x->vf_bsp_amp_milli, sizeof(_x->vf_bsp_amp_milli), CO_DT_UINT32, CO_ACCESS_RW);
    _x->objs[31] = (co_obj_t){ .index = 0x2032u, .sub_number = 4, .subs = _x->sub_2032 };
}


co_obj_t* cia402_drive_od_objs(cia402_drive_od_t* _x){
    return _x ? _x->objs : (co_obj_t*)0;
}
