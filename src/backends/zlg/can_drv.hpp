#ifndef CAN_DEV_H
#define CAN_DEV_H

#include <cstring>
#include <string>
#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace can_drv{
    typedef union {
        struct can_frame std_frame;
        struct canfd_frame fd_frame;
    }can_msg_t;

    static void can_fd_msg_copy(const can_msg_t& _src, can_msg_t& _dest){
        _dest.fd_frame.can_id = _src.fd_frame.can_id;
        _dest.fd_frame.flags = _src.fd_frame.flags;
        _dest.fd_frame.len = _src.fd_frame.len;
        _dest.fd_frame.__res0 = _src.fd_frame.__res0;
        _dest.fd_frame.__res1 = _src.fd_frame.__res1;
        memcpy(_dest.fd_frame.data, _src.fd_frame.data, _src.fd_frame.len);
    }

     static void can_msg_copy(const can_msg_t& _src, can_msg_t& _dest){
        _dest.std_frame.can_id = _src.std_frame.can_id;
        _dest.std_frame.len = _src.std_frame.len;
        _dest.std_frame.__res0 = _src.std_frame.__res0;
        _dest.std_frame.__pad = _src.std_frame.__pad;
        _dest.std_frame.len8_dlc = _src.std_frame.len8_dlc;
        memcpy(_dest.std_frame.data, _src.std_frame.data, CAN_MAX_DLC);
    }

    class CanDriver {
    public:
        virtual ~CanDriver() = default;
        
        virtual int32_t config(bool _is_fd, uint32_t _bit_rate) = 0;

        virtual std::string getDevInfo() = 0;

        virtual int32_t readSync(can_msg_t& _msg, uint32_t _timeout) = 0;
        virtual int32_t writeSync(const can_msg_t& _msg, uint32_t _timeout) = 0;

        virtual int32_t readAsync(can_msg_t& _msg) = 0;
        virtual int32_t writeAsync(const can_msg_t& _msg) = 0;
    };

}

#endif
