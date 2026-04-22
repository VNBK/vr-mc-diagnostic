#include "hal_can_zlg.hpp"
#include "zlg_can_drv.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

namespace vrmc {

/* --- impl ------------------------------------------------------------ */

struct ZlgImpl
{
    hal_can_t                                base{};         /* must be first  */
    std::shared_ptr<can_drv::ZlgCanDrv>      drv;
    std::shared_ptr<can_drv::ZlgCanChannel>  ch;
    uint32_t                                 ab_bps  = 1'000'000;
    uint32_t                                 fd_bps  = 0;
    bool                                     fd_mode = false;

    hal_can_rx_fn_t                          rx_cb   = nullptr;
    hal_can_tx_fn_t                          tx_cb   = nullptr;
    void*                                    cb_arg  = nullptr;

    std::atomic<bool>                        running{false};
    std::thread                              rxThread;
};

#define _self(x)   reinterpret_cast<ZlgImpl*>(x)

/* --- helpers --------------------------------------------------------- */

static void to_zlg_msg(const can_msg_t& src, can_drv::can_msg_t& dst)
{
    /* The vendor's can_drv::can_msg_t is a union of struct can_frame +
     * struct canfd_frame. We treat everything as canfd; the vendor
     * library reads .len / .can_id / .data uniformly. */
    std::memset(&dst, 0, sizeof(dst));
    dst.fd_frame.can_id = src.msg_id;
    dst.fd_frame.len    = src.len;
    if (src.flags & CAN_FLAG_EXTENDED_ID){
        dst.fd_frame.can_id |= 0x80000000u;     /* CAN_EFF_FLAG */
    }
    if (src.flags & CAN_FLAG_BITRATE_SWITCH){
        dst.fd_frame.flags |= 0x01;             /* CANFD_BRS    */
    }
    if (src.flags & CAN_FLAG_ERROR_STATE){
        dst.fd_frame.flags |= 0x02;             /* CANFD_ESI    */
    }
    const size_t n = (src.flags & CAN_FLAG_FD)
                         ? std::min<size_t>(src.len, CAN_FD_MSG_LENGTH_DEFAULT)
                         : std::min<size_t>(src.len, CAN_MSG_LENGTH_DEFAULT);
    std::memcpy(dst.fd_frame.data, src.msg.msg_fd, n);
}

static void from_zlg_msg(const can_drv::can_msg_t& src, can_msg_t& dst, bool fd)
{
    can_msg_reset(&dst);
    dst.msg_id = src.fd_frame.can_id & 0x1FFFFFFFu;
    if (src.fd_frame.can_id & 0x80000000u){
        dst.flags |= CAN_FLAG_EXTENDED_ID;
    }
    if (fd){
        dst.flags |= CAN_FLAG_FD;
        if (src.fd_frame.flags & 0x01){ dst.flags |= CAN_FLAG_BITRATE_SWITCH; }
        if (src.fd_frame.flags & 0x02){ dst.flags |= CAN_FLAG_ERROR_STATE;    }
    }
    dst.len = src.fd_frame.len;
    const size_t n = std::min<size_t>(dst.len, CAN_FD_MSG_LENGTH_DEFAULT);
    std::memcpy(dst.msg.msg_fd, src.fd_frame.data, n);
}

/* --- vtable ---------------------------------------------------------- */

static int32_t zlg_init(hal_can_t* h, uint32_t bitrate, uint32_t fd_bitrate)
{
    auto* s = _self(h);
    if (!s->ch){ return -1; }
    s->ab_bps  = bitrate ? bitrate : s->ab_bps;
    s->fd_bps  = fd_bitrate;
    s->fd_mode = (fd_bitrate > 0);
    return s->ch->config(/*_fd=*/s->fd_mode, /*_extended=*/true,
                         s->ab_bps, s->fd_mode ? s->fd_bps : s->ab_bps);
}

static int32_t zlg_free(hal_can_t* h)
{
    auto* s = _self(h);
    s->running.store(false);
    if (s->rxThread.joinable()){ s->rxThread.join(); }
    s->ch.reset();
    s->drv.reset();
    return 0;
}

static int32_t zlg_start(hal_can_t* h)
{
    auto* s = _self(h);
    if (s->running.load()){ return 0; }
    s->running.store(true);
    s->rxThread = std::thread([s]{
        while (s->running.load()){
            can_drv::can_msg_t msg{};
            int32_t rc = s->ch->readSync(msg, /*_timeout_ms=*/10);
            if (rc > 0 && s->rx_cb){
                can_msg_t out;
                from_zlg_msg(msg, out, s->fd_mode);
                s->rx_cb(&out, s->cb_arg);
            }
        }
    });
    return 0;
}

static int32_t zlg_stop(hal_can_t* h)
{
    auto* s = _self(h);
    s->running.store(false);
    if (s->rxThread.joinable()){ s->rxThread.join(); }
    return 0;
}

static int32_t zlg_write(hal_can_t* h, const can_msg_t* msg)
{
    auto* s = _self(h);
    if (!s->ch || !msg){ return -1; }
    can_drv::can_msg_t z{};
    to_zlg_msg(*msg, z);
    const int32_t rc = s->ch->writeAsync(z);
    if (rc >= 0 && s->tx_cb){ s->tx_cb(s->cb_arg); }
    return (rc >= 0) ? 0 : -1;
}

static int32_t zlg_set_cb(hal_can_t* h, hal_can_rx_fn_t rx,
                          hal_can_tx_fn_t tx, void* arg)
{
    auto* s = _self(h);
    s->rx_cb  = rx;
    s->tx_cb  = tx;
    s->cb_arg = arg;
    return 0;
}

static int32_t zlg_set_filter(hal_can_t* /*h*/, uint8_t /*idx*/,
                              uint32_t /*id*/, uint32_t /*mask*/)
{
    /* The wrapped library exposes filter setters but we accept everything
     * for the diagnostic — keep symmetry with hal_can_udp behaviour. */
    return 0;
}

static int32_t zlg_get_bus_status(hal_can_t* /*h*/, uint32_t* out)
{
    if (out){ *out = CAN_BUS_OK; }
    return 0;
}

static int32_t zlg_recover(hal_can_t* /*h*/) { return 0; }

static const hal_can_proc_t s_zlg_proc = {
    .init             = zlg_init,
    .free             = zlg_free,
    .start            = zlg_start,
    .stop             = zlg_stop,
    .write            = zlg_write,
    .set_cb           = zlg_set_cb,
    .set_filter       = zlg_set_filter,
    .get_bus_status   = zlg_get_bus_status,
    .bus_off_recovery = zlg_recover,
};

/* --- public API ------------------------------------------------------ */

hal_can_t* hal_can_zlg_create(const std::string& lib_path,
                              uint32_t            channel,
                              uint32_t            bitrate_bps,
                              uint32_t            fd_bps)
{
    auto drv = can_drv::ZlgCanDrv::create(lib_path);
    if (!drv){ return nullptr; }
    auto ch  = drv->getChannel(static_cast<uint8_t>(channel));
    if (!ch){ return nullptr; }

    auto* s = new ZlgImpl();
    s->base.proc = &s_zlg_proc;
    s->drv       = drv;
    s->ch        = ch;
    s->ab_bps    = bitrate_bps;
    s->fd_bps    = fd_bps;
    s->fd_mode   = (fd_bps > 0);
    if (zlg_init(&s->base, bitrate_bps, fd_bps) != 0){
        delete s;
        return nullptr;
    }
    return &s->base;
}

void hal_can_zlg_destroy(hal_can_t* h)
{
    if (!h){ return; }
    zlg_free(h);
    delete _self(h);
}

}  // namespace vrmc
