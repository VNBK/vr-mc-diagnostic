#include <dlfcn.h>
#include <iostream>
#include <sstream>
#include "controlcanfd.h"
#include "logger.hpp"
#include "timer.h"
#include "zlg_can_drv.hpp"

#define TAG "WaveShare Can Driver"

namespace can_drv
{
std::shared_ptr<ZlgCanDrv> ZlgCanDrv::create(const std::string &_lib_path,
                                             uint32_t _dev_index)
{
    auto drv = std::shared_ptr<ZlgCanDrv>(new ZlgCanDrv(_lib_path, _dev_index));
    if (!drv)
    {
        LOG_ERROR(TAG, "Could NOT create WaveShare Can Driver");
        return NULL;
    }

    if (drv->initialize())
    {
        LOG_INFO(TAG, "Created WaveShare Can Driver SUCCESS");
        return drv;
    }

    return NULL;
}

uint32_t ZlgCanDrv::getChannelSupport()
{
    return 2;
}

ZlgCanDrv::ZlgCanDrv(const std::string &_lib_path, uint32_t _dev_index) 
    : m_path(_lib_path),
    m_dev_index(_dev_index),
    m_dev_handle(NULL),
    m_dl(NULL)
{
}

bool ZlgCanDrv::initialize()
{
    if (!loadLib())
    {
        return false;
    }

    m_dev_handle = zcan_open_device(USBCANFD_200U, m_dev_index, 0);
    if (m_dev_handle == INVALID_DEVICE_HANDLE)
    {
        LOG_ERROR(TAG, "ZCan failed to open device");
        return false;
    }
    return true;
}

bool ZlgCanDrv::loadLib()
{
    m_dl = dlopen(m_path.c_str(), RTLD_NOW);
    if (!m_dl)
    {
        LOG_ERROR(TAG, "Could Not load library");
        return false;
    }

    zcan_open_device = (pZCAN_OpenDevice)dlsym(m_dl, "ZCAN_OpenDevice");
    zcan_close_device = (pZCAN_CloseDevice)dlsym(m_dl, "ZCAN_CloseDevice");
    zcan_get_device_inf = (pZCAN_GetDeviceInf)dlsym(m_dl, "ZCAN_GetDeviceInf");
    zcan_is_device_online = (pZCAN_IsDeviceOnLine)dlsym(m_dl, "ZCAN_IsDeviceOnLine");
    zcan_init_can = (pZCAN_InitCAN)dlsym(m_dl, "ZCAN_InitCAN");
    zcan_start_can = (pZCAN_StartCAN)dlsym(m_dl, "ZCAN_StartCAN");
    zcan_reset_can = (pZCAN_ResetCAN)dlsym(m_dl, "ZCAN_ResetCAN");
    zcan_clear_buffer = (pZCAN_ClearBuffer)dlsym(m_dl, "ZCAN_ClearBuffer");
    zcan_get_receive_num = (pZCAN_GetReceiveNum)dlsym(m_dl, "ZCAN_GetReceiveNum");
    zcan_transmit = (pZCAN_Transmit)dlsym(m_dl, "ZCAN_Transmit");
    zcan_receive = (pZCAN_Receive)dlsym(m_dl, "ZCAN_Receive");
    zcan_transmit_fd = (pZCAN_TransmitFD)dlsym(m_dl, "ZCAN_TransmitFD");
    zcan_receive_fd = (pZCAN_ReceiveFD)dlsym(m_dl, "ZCAN_ReceiveFD");
    get_iproperty = (pGetIProperty)dlsym(m_dl, "GetIProperty");
    release_iproperty = (pReleaseIProperty)dlsym(m_dl, "ReleaseIProperty");
    zcan_set_abit_baud = (pZCAN_SetAbitBaud)dlsym(m_dl, "ZCAN_SetAbitBaud");
    zcan_set_dbit_baud = (pZCAN_SetDbitBaud)dlsym(m_dl, "ZCAN_SetDbitBaud");
    zcan_set_canfd_standard = (pZCAN_SetCANFDStandard)dlsym(m_dl, "ZCAN_SetCANFDStandard");
    zcan_set_resistance_enable = (pZCAN_SetResistanceEnable)dlsym(m_dl, "ZCAN_SetResistanceEnable");
    zcan_set_baud_rate_custom = (pZCAN_SetBaudRateCustom)dlsym(m_dl, "ZCAN_SetBaudRateCustom");
    zcan_clear_filter = (pZCAN_ClearFilter)dlsym(m_dl, "ZCAN_ClearFilter");
    zcan_ack_filter = (pZCAN_AckFilter)dlsym(m_dl, "ZCAN_AckFilter");
    zcan_set_filter_mode = (pZCAN_SetFilterMode)dlsym(m_dl, "ZCAN_SetFilterMode");
    zcan_set_filter_start_id = (pZCAN_SetFilterStartID)dlsym(m_dl, "ZCAN_SetFilterStartID");
    zcan_set_filter_end_id = (pZCAN_SetFilterEndID)dlsym(m_dl, "ZCAN_SetFilterEndID");

    if (!zcan_open_device || !zcan_close_device || !zcan_get_device_inf || !zcan_is_device_online ||
        !zcan_init_can || !zcan_start_can || !zcan_reset_can || !zcan_clear_buffer ||
        !zcan_get_receive_num || !zcan_transmit || !zcan_receive || !zcan_transmit_fd ||
        !zcan_receive_fd || !get_iproperty || !release_iproperty || !zcan_set_abit_baud ||
        !zcan_set_dbit_baud || !zcan_set_canfd_standard || !zcan_set_resistance_enable ||
        !zcan_set_baud_rate_custom || !zcan_clear_filter || !zcan_ack_filter ||
        !zcan_set_filter_mode || !zcan_set_filter_start_id || !zcan_set_filter_end_id)
    {
        LOG_ERROR(TAG, "Load library FAILURE");
        dlclose(m_dl);
        m_dl = nullptr;

        return false;
    }

    LOG_DEBUG(TAG, "Load library SUCCESS");
    return true;
}

void ZlgCanDrv::unloadLib()
{
    if (m_dl)
    {
        dlclose(m_dl);
        m_dl = nullptr;
    }
}

ZlgCanDrv::~ZlgCanDrv()
{
    unloadLib();
    if (m_dev_handle)
    {
        zcan_close_device(m_dev_handle);
        m_dev_handle = NULL;
    }
}

std::string ZlgCanDrv::getDevInfo()
{
    if (m_dev_handle)
    {
        ZCAN_DEVICE_INFO dev_info;
        zcan_get_device_inf(m_dev_handle, &dev_info);

        std::ostringstream info;

        info << R"(SN: )" << dev_info.str_Serial_Num;
        info << R"(,Type: )" << dev_info.str_hw_Type;
        info << R"(,Hw version: )" << dev_info.hw_Version;
        info << R"(,Sw version: )" << dev_info.fw_Version;
        info << R"(,Dr version: )" << dev_info.dr_Version;
        info << R"(,Dr version: )" << dev_info.in_Version;
        info << R"(,IRQ number: )" << dev_info.irq_Num;
        info << R"(,Can number: )" << dev_info.can_Num;
        info << R"(.)";

        return info.str().c_str();
    }

    return "";
}

std::shared_ptr<ZlgCanChannel> ZlgCanDrv::getChannel(uint8_t index){
    if(index >= this->getChannelSupport()){
        return nullptr;
    }
    auto channel = std::make_shared<ZlgCanChannel>(index, std::shared_ptr<ZlgCanDrv>(this));
    if(!channel){
        LOG_ERROR(TAG, "Created Zlg Can Channel FAILURE");
        return NULL;
    }
    return channel;
}


/******************************** ZlgCanChannel ****************************/

/// @brief Contructor ZlgCanChannel
/// @param _parent
ZlgCanChannel::ZlgCanChannel(uint8_t _index, std::shared_ptr<ZlgCanDrv> _parent) : m_index(_index),
    m_parent(_parent),
    m_handle(nullptr),
    m_fd(TYPE_CANFD),
    m_extended(0)
{
}

ZlgCanChannel::~ZlgCanChannel()
{
    if (m_parent && m_handle)
    {
        m_parent->zcan_reset_can(m_handle);
    }
    m_handle = nullptr;
    m_parent = nullptr;
}

int32_t ZlgCanChannel::config(bool _is_fd, uint32_t _bit_rate)
{
    return 0;
}

std::string ZlgCanChannel::getDevInfo()
{
    return m_parent->getDevInfo();
}

int32_t ZlgCanChannel::readSync(can_msg_t &_msg, uint32_t _timeout)
{
    WaitTimer timeout(_timeout);
    while(m_parent->zcan_get_receive_num(m_handle, m_fd) < 0 || timeout.getRemainTime())
    {
        if (m_fd)
        {
            ZCAN_ReceiveFD_Data data;
            uint32_t num = m_parent->zcan_receive_fd(m_handle, &data, 1, _timeout);
            if (num > 0)
            {
                can_fd_msg_copy((can_msg_t &)data.frame, _msg);
                _msg.fd_frame.can_id = GET_ID(data.frame.can_id);
                return data.frame.len;
            }
        }
    }
    return 0;
}

int32_t ZlgCanChannel::writeSync(const can_msg_t &_msg, uint32_t _timeout)
{
    if (m_fd)
    {
        ZCAN_TransmitFD_Data data;

        data.transmit_type = 0;
        can_fd_msg_copy(_msg, (can_msg_t &)data.frame);
        data.frame.can_id = MAKE_CAN_ID(_msg.fd_frame.can_id, m_extended, 0, 0);

        if (STATUS_OK != m_parent->zcan_transmit_fd(m_handle, &data, 1))
        {
            LOG_ERROR(TAG, "Transmit can message FAILURE");
            return 0;
        }
        return _msg.fd_frame.len;
    }
    return 0;
}

int32_t ZlgCanChannel::readAsync(can_msg_t &_msg)
{
    if (m_fd) {
        ZCAN_ReceiveFD_Data data;
        uint32_t num = m_parent->zcan_receive_fd(m_handle, &data, 1, 10);
        if (num > 0)
        {
            can_fd_msg_copy((can_msg_t &)data.frame, _msg);
            _msg.fd_frame.can_id = GET_ID(data.frame.can_id);
            return data.frame.len;
        }
    }
    return -1;
}

int32_t ZlgCanChannel::writeAsync(const can_msg_t &_msg)
{
    if (m_fd) {
        ZCAN_TransmitFD_Data data;
        data.transmit_type = 0;
        can_fd_msg_copy(_msg, (can_msg_t &)data.frame);
        data.frame.can_id = MAKE_CAN_ID(_msg.fd_frame.can_id, m_extended, 0, 0);
        if (STATUS_OK != m_parent->zcan_transmit_fd(m_handle, &data, 1))
        {
            LOG_ERROR(TAG, "Transmit can message FAILURE");
            return -1;
        }
        return _msg.fd_frame.len;
    }
    return -1;
}

int32_t ZlgCanChannel::config(bool _fd, bool _extented, uint32_t _abit_rate, uint32_t _dbit_rate)
{
    m_fd = _fd ? TYPE_CANFD : TYPE_CAN;
    m_extended = _extented;

    if (m_parent->zcan_set_abit_baud(m_parent->m_dev_handle, m_index, _abit_rate) != STATUS_OK || 
        m_parent->zcan_set_dbit_baud(m_parent->m_dev_handle, m_index, _dbit_rate) != STATUS_OK) {
        LOG_ERROR(TAG, "Set BitRate FAILURE");
    }

    ZCAN_CHANNEL_INIT_CONFIG config{};
    config.can_type = m_fd;
    config.canfd.acc_code = 0;
    config.canfd.acc_mask = 0xFFFFFFFF;
    config.canfd.filter = 1;
    config.canfd.mode = 0;
    config.canfd.brp = 0;

    m_handle = m_parent->zcan_init_can(m_parent->m_dev_handle, m_index, &config);
    if (m_handle == INVALID_CHANNEL_HANDLE)
    {
        LOG_ERROR(TAG, "Failed to initialize CAN channel %d", m_index);
        return -1;
    }

    m_parent->zcan_start_can(m_handle);
    return 0;
}
} // namespace can_drv
