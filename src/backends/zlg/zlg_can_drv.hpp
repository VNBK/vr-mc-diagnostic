#pragma once

#include <memory>
#include "can_drv.hpp"
#include "controlcanfd.h"

namespace can_drv
{
    class ZlgCanDrv;

    class ZlgCanChannel : public CanDriver
    {
    public:
        ZlgCanChannel(uint8_t _index, std::shared_ptr<ZlgCanDrv> _parent);
        virtual ~ZlgCanChannel();

        virtual int32_t config(bool _is_fd, uint32_t _bit_rate) override;
        virtual std::string getDevInfo() override;

        virtual int32_t readSync(can_msg_t &_msg, uint32_t _timeout) override;
        virtual int32_t writeSync(const can_msg_t &_msg, uint32_t _timeout) override;

        virtual int32_t readAsync(can_msg_t &_msg) override;
        virtual int32_t writeAsync(const can_msg_t &_msg) override;

        int32_t config(bool _fd = true, bool _extented = true, uint32_t _abit_rate = 1E6, uint32_t _dbit_rate = 1E6);

    private:
        std::shared_ptr<ZlgCanDrv> m_parent;
        uint8_t m_index;
        CHANNEL_HANDLE m_handle;
        uint8_t m_fd;
        uint8_t m_extended;
    };

    class ZlgCanDrv{
        public:
            static const uint32_t m_dev_index_default = 0;

            static std::shared_ptr<ZlgCanDrv> create(const std::string& _lib_path, 
                                                    uint32_t _dev_index = m_dev_index_default);
            static uint32_t getChannelSupport();

            ~ZlgCanDrv();

            std::string getDevInfo();

            std::shared_ptr<ZlgCanChannel> getChannel(uint8_t index = 0);

            friend class ZlgCanChannel;
        private:
            ZlgCanDrv(const std::string& _lib_path, uint32_t _dev_index);
            
            bool initialize();

            bool loadLib();
            void unloadLib();

                // Function pointers
            pZCAN_OpenDevice zcan_open_device;
            pZCAN_CloseDevice zcan_close_device;
            pZCAN_GetDeviceInf zcan_get_device_inf;
            pZCAN_IsDeviceOnLine zcan_is_device_online;
            pZCAN_InitCAN zcan_init_can;
            pZCAN_StartCAN zcan_start_can;
            pZCAN_ResetCAN zcan_reset_can;
            pZCAN_ClearBuffer zcan_clear_buffer;
            pZCAN_GetReceiveNum zcan_get_receive_num;
            pZCAN_Transmit zcan_transmit;
            pZCAN_Receive zcan_receive;
            pZCAN_TransmitFD zcan_transmit_fd;
            pZCAN_ReceiveFD zcan_receive_fd;
            pGetIProperty get_iproperty;
            pReleaseIProperty release_iproperty;
            pZCAN_SetAbitBaud zcan_set_abit_baud;
            pZCAN_SetDbitBaud zcan_set_dbit_baud;
            pZCAN_SetCANFDStandard zcan_set_canfd_standard;
            pZCAN_SetResistanceEnable zcan_set_resistance_enable;
            pZCAN_SetBaudRateCustom zcan_set_baud_rate_custom;
            pZCAN_ClearFilter zcan_clear_filter;
            pZCAN_AckFilter zcan_ack_filter;
            pZCAN_SetFilterMode zcan_set_filter_mode;
            pZCAN_SetFilterStartID zcan_set_filter_start_id;
            pZCAN_SetFilterEndID zcan_set_filter_end_id;

            std::string m_path;
            uint32_t m_dev_index;
            DEVICE_HANDLE m_dev_handle;

            void* m_dl;
    };
    
} // namespace can_drv
