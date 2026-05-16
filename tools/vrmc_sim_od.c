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
    _x->motor_rated_torque_uNm = CIA402_DRIVE_OD_DEFAULT_RATED_TORQUE_uNm;
    _x->current_actual_mA      = 0;
    _x->home_offset            = 0;
    _x->pos_limit_min          = INT32_MIN;
    _x->pos_limit_max          = INT32_MAX;
    _x->max_motor_speed        = UINT32_MAX;   /* no clamp by default */
    _x->n_607D_subs            = 2u;
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

    /* 0x6076 Motor rated torque (RW U32, μNm) */
    install_sub(&_x->sub_6076[0], &_x->motor_rated_torque_uNm,
                sizeof(_x->motor_rated_torque_uNm), CO_DT_UINT32,
                CO_ACCESS_RW);
    _x->objs[1] = (co_obj_t){ .index = 0x6076u, .sub_number = 1,
                               .subs = _x->sub_6076 };

    /* 0x6078 Current actual value (RO I16, PDO-TX mappable) */
    install_sub(&_x->sub_6078[0], &_x->current_actual_mA,
                sizeof(_x->current_actual_mA), CO_DT_INT16,
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
}


co_obj_t* cia402_drive_od_objs(cia402_drive_od_t* _x){
    return _x ? _x->objs : (co_obj_t*)0;
}
