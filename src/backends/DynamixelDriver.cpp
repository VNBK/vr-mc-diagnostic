#include "DynamixelDriver.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>

namespace vrmc {

/* -------------------------------------------------------------------- *
 * Protocol 2.0 framing + CRC
 * -------------------------------------------------------------------- */

namespace {

constexpr uint8_t kInstPing  = 0x01;
constexpr uint8_t kInstRead  = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstStatus = 0x55;

/* Dynamixel-X control-table addresses. Same layout across XL-330, XL-430,
 * XM-430, XH-430 (the common workshop servos). Torque/Current scaling
 * varies per model — we treat the Present Current register as raw units
 * and let the caller scale via A_per_Nm. */
constexpr uint16_t kAddrOpMode      = 11;    /* 1 byte  */
constexpr uint16_t kAddrTorqueEn    = 64;    /* 1 byte  */
constexpr uint16_t kAddrLED         = 65;    /* 1 byte  */
constexpr uint16_t kAddrGoalCurrent = 102;   /* 2 bytes */
constexpr uint16_t kAddrGoalVel     = 104;   /* 4 bytes */
constexpr uint16_t kAddrGoalPos     = 116;   /* 4 bytes */
constexpr uint16_t kAddrPresentCurrent = 126;/* 2 bytes */
constexpr uint16_t kAddrPresentVel  = 128;   /* 4 bytes */
constexpr uint16_t kAddrPresentPos  = 132;   /* 4 bytes */
constexpr uint16_t kAddrPresentTemp = 146;   /* 1 byte  */
constexpr uint16_t kAddrHwError     = 70;    /* 1 byte  */

/* Operating-mode values (control table 11). */
constexpr uint8_t  kModeCurrent  = 0;
constexpr uint8_t  kModeVelocity = 1;
constexpr uint8_t  kModePosition = 3;
constexpr uint8_t  kModeExtPos   = 4;

/* Encoder: 4096 counts per revolution. */
constexpr double kCountsPerRev = 4096.0;
constexpr double kRadPerCount  = (2.0 * M_PI) / kCountsPerRev;

/* Velocity: 0.229 RPM per raw unit. */
constexpr double kRpmPerUnit   = 0.229;
constexpr double kRadPerSecPerUnit = kRpmPerUnit * 2.0 * M_PI / 60.0;

/* Temperature: direct degC. Hardware error bitmask lives at 0x46. */
constexpr uint8_t kHwErrOverload    = 1u << 5;
constexpr uint8_t kHwErrElectShock  = 1u << 4;
constexpr uint8_t kHwErrOverheating = 1u << 2;
constexpr uint8_t kHwErrInputVolt   = 1u << 0;


/* CRC-16 polynomial 0x8005, as specified in the Dynamixel 2.0 manual
 * (identical to the ROBOTIS SDK's table). Byte-wise update. */
constexpr std::array<uint16_t, 256> makeCrcTable()
{
    std::array<uint16_t, 256> t{};
    for (int i = 0; i < 256; ++i){
        uint16_t c = uint16_t(i) << 8;
        for (int j = 0; j < 8; ++j){
            c = (c & 0x8000u) ? uint16_t((c << 1) ^ 0x8005u) : uint16_t(c << 1);
        }
        t[i] = c;
    }
    return t;
}
static const std::array<uint16_t, 256> kCrcTable = makeCrcTable();

uint16_t crcAccum(uint16_t crc, const uint8_t* p, size_t n)
{
    for (size_t i = 0; i < n; ++i){
        const uint8_t x = uint8_t(((crc >> 8) ^ p[i]) & 0xFF);
        crc = uint16_t((crc << 8) ^ kCrcTable[x]);
    }
    return crc;
}

/* Byte-stuffing per manual: after the third 0xFD in "FF FF FD" the next
 * must be stuffed. Protocol-side stuffing matters for pathological
 * payloads; control-table reads / writes rarely hit it. We still apply
 * it correctly to be safe. */
void stuffInto(std::vector<uint8_t>& out, const uint8_t* src, size_t n)
{
    int ffCount = 0;
    for (size_t i = 0; i < n; ++i){
        const uint8_t b = src[i];
        out.push_back(b);
        if (b == 0xFF){
            ++ffCount;
            continue;
        }
        if (ffCount >= 2 && b == 0xFD){
            out.push_back(0xFD);   /* stuff */
            ffCount = 0;
            continue;
        }
        ffCount = 0;
    }
}

/* Reverse of stuffInto — drops one 0xFD after every "FF FF FD". */
size_t unstuff(uint8_t* data, size_t n)
{
    size_t w = 0;
    int ffCount = 0;
    for (size_t r = 0; r < n; ++r){
        const uint8_t b = data[r];
        data[w++] = b;
        if (b == 0xFF){
            ++ffCount;
            continue;
        }
        if (ffCount >= 2 && b == 0xFD && r + 1 < n && data[r + 1] == 0xFD){
            ++r;              /* swallow the stuff byte */
            ffCount = 0;
            continue;
        }
        ffCount = 0;
    }
    return w;
}

/* Build a full Protocol 2.0 frame: FF FF FD 00 ID LEN_L LEN_H INST
 * <stuffed payload> CRC_L CRC_H. @p payload excludes INST but includes
 * any per-instruction bytes. */
std::vector<uint8_t> buildPacket(uint8_t id, uint8_t inst,
                                 const uint8_t* payload, size_t pay_n)
{
    std::vector<uint8_t> stuffed;
    stuffed.reserve(pay_n + 8);
    stuffInto(stuffed, payload, pay_n);

    /* LEN field covers: INST + stuffed payload + CRC (2 bytes). */
    const uint16_t len = uint16_t(1 + stuffed.size() + 2);

    std::vector<uint8_t> pkt;
    pkt.reserve(10 + stuffed.size());
    pkt.push_back(0xFF); pkt.push_back(0xFF); pkt.push_back(0xFD);
    pkt.push_back(0x00);
    pkt.push_back(id);
    pkt.push_back(uint8_t(len & 0xFF));
    pkt.push_back(uint8_t(len >> 8));
    pkt.push_back(inst);
    pkt.insert(pkt.end(), stuffed.begin(), stuffed.end());

    /* CRC over everything except the CRC field itself. */
    const uint16_t c = crcAccum(0, pkt.data(), pkt.size());
    pkt.push_back(uint8_t(c & 0xFF));
    pkt.push_back(uint8_t(c >> 8));
    return pkt;
}

bool waitReadable(int fd, int timeout_ms)
{
    struct pollfd p{}; p.fd = fd; p.events = POLLIN;
    const int rv = ::poll(&p, 1, timeout_ms);
    return rv > 0 && (p.revents & POLLIN);
}

}  // namespace


/* -------------------------------------------------------------------- *
 * DynamixelBus — serial-port owner
 * -------------------------------------------------------------------- */

DynamixelBus* DynamixelBus::open(const std::string& device, uint32_t baud,
                                 std::string* err)
{
    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0){
        if (err){ *err = "open(" + device + ") failed"; }
        return nullptr;
    }
    /* Non-blocking reads — we poll+read explicitly. */
    termios tio{};
    if (tcgetattr(fd, &tio) != 0){
        if (err){ *err = "tcgetattr failed"; }
        ::close(fd);
        return nullptr;
    }
    cfmakeraw(&tio);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |=  CS8;
    tio.c_cflag &= ~PARENB;        /* 8N1 */
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag |=  CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    speed_t s = B1000000;
    switch (baud){
    case  57600:  s = B57600;  break;
    case 115200:  s = B115200; break;
    case 230400:  s = B230400; break;
    case 460800:  s = B460800; break;
    case 500000:  s = B500000; break;
    case 576000:  s = B576000; break;
    case 921600:  s = B921600; break;
    case 1000000: s = B1000000;break;
    case 2000000: s = B2000000;break;
    case 3000000: s = B3000000;break;
    case 4000000: s = B4000000;break;
    default: break;            /* fall through: 1 Mbps */
    }
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);

    if (tcsetattr(fd, TCSANOW, &tio) != 0){
        if (err){ *err = "tcsetattr failed"; }
        ::close(fd);
        return nullptr;
    }
    tcflush(fd, TCIOFLUSH);

    auto* self = new DynamixelBus();
    self->m_fd = fd;
    return self;
}

DynamixelBus::~DynamixelBus()
{
    if (m_fd >= 0){ ::close(m_fd); m_fd = -1; }
}

/* Send @p pkt and wait for one Status packet from @p expectId. Copies
 * error code + params into @p outParams / @p outLen. The caller owns a
 * mutex if multiple threads share the bus; for this diagnostic the
 * worker thread is the only caller so we skip the lock. */
static int txRx(int fd, uint8_t expectId, uint8_t* rxbuf,
                const std::vector<uint8_t>& pkt,
                uint8_t* outError, uint8_t* outParams, int* outLen,
                int timeout_ms = 50)
{
    if (fd < 0){ return -1; }
    tcflush(fd, TCIOFLUSH);
    const ssize_t w = ::write(fd, pkt.data(), pkt.size());
    if (w < 0 || size_t(w) != pkt.size()){ return -2; }

    /* Drain echo if half-duplex tty loops TX back. We consume anything
     * that matches the outgoing bytes until we see an inbound status. */
    size_t have = 0;
    const auto deadline_ms = timeout_ms;
    auto t_rem = deadline_ms;
    while (have < 11 || (have >= 7 && /* we have at least header + len */
           have < size_t(11 + (rxbuf[5] | (rxbuf[6] << 8))))){
        if (!waitReadable(fd, t_rem)){ return -3; }
        uint8_t buf[256];
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n <= 0){ continue; }
        if (have + size_t(n) > 256){ return -4; }
        memcpy(rxbuf + have, buf, size_t(n));
        have += size_t(n);
        /* Resync on the FF FF FD 00 header. */
        size_t p = 0;
        while (p + 4 <= have){
            if (rxbuf[p] == 0xFF && rxbuf[p+1] == 0xFF &&
                rxbuf[p+2] == 0xFD && rxbuf[p+3] == 0x00){
                break;
            }
            ++p;
        }
        if (p > 0){
            memmove(rxbuf, rxbuf + p, have - p);
            have -= p;
        }
        if (have >= 7){
            const uint16_t len = uint16_t(rxbuf[5] | (rxbuf[6] << 8));
            if (have >= size_t(7) + len){ break; }
        }
    }
    /* Validate status packet. */
    if (have < 11){ return -5; }
    const uint16_t len = uint16_t(rxbuf[5] | (rxbuf[6] << 8));
    const size_t frame_end = 7 + len;
    if (frame_end > have){ return -6; }
    if (rxbuf[7] != kInstStatus){ return -7; }
    if (rxbuf[4] != expectId){ return -8; }

    /* Verify CRC. */
    const uint16_t got = uint16_t(rxbuf[frame_end - 2] |
                                  (rxbuf[frame_end - 1] << 8));
    const uint16_t calc = crcAccum(0, rxbuf, frame_end - 2);
    if (got != calc){ return -9; }

    const uint8_t errByte = rxbuf[8];
    if (outError){ *outError = errByte; }

    const int payload_n = int(len) - 4;   /* subtract INST + ERROR + CRC */
    if (payload_n > 0 && outParams){
        /* Un-stuff the payload. */
        memcpy(outParams, rxbuf + 9, size_t(payload_n));
        const size_t real = unstuff(outParams, size_t(payload_n));
        if (outLen){ *outLen = int(real); }
    } else if (outLen){
        *outLen = 0;
    }
    return 0;
}

int DynamixelBus::ping(uint8_t id)
{
    auto pkt = buildPacket(id, kInstPing, nullptr, 0);
    uint8_t params[16];
    int n = 0;
    return txRx(m_fd, id, m_rxbuf, pkt, nullptr, params, &n, 80);
}

int DynamixelBus::read(uint8_t id, uint16_t addr, uint16_t len, uint8_t* buf)
{
    const uint8_t payload[4] = {
        uint8_t(addr & 0xFF), uint8_t(addr >> 8),
        uint8_t(len  & 0xFF), uint8_t(len  >> 8),
    };
    auto pkt = buildPacket(id, kInstRead, payload, sizeof(payload));
    uint8_t params[256]; int n = 0;
    const int rc = txRx(m_fd, id, m_rxbuf, pkt, nullptr, params, &n, 80);
    if (rc != 0){ return rc; }
    if (n < int(len)){ return -10; }
    memcpy(buf, params, len);
    return 0;
}

int DynamixelBus::write(uint8_t id, uint16_t addr, uint16_t len,
                        const uint8_t* data)
{
    std::vector<uint8_t> payload;
    payload.reserve(2 + len);
    payload.push_back(uint8_t(addr & 0xFF));
    payload.push_back(uint8_t(addr >> 8));
    payload.insert(payload.end(), data, data + len);
    auto pkt = buildPacket(id, kInstWrite, payload.data(), payload.size());
    uint8_t dummy; int n = 0;
    return txRx(m_fd, id, m_rxbuf, pkt, &dummy, nullptr, &n, 80);
}


/* -------------------------------------------------------------------- *
 * motor_drive_intf_t adapter
 * -------------------------------------------------------------------- */

namespace {

struct DxlIntf {
    motor_drive_intf_t base;
    DynamixelBus*      bus;
    uint8_t            id;
    float              A_per_Nm;
    motor_intf_mode_t  bringup_mode;
    char               name[32];
    motor_intf_mode_t  last_mode;   /* cached to avoid spurious SDO reads */
};

static DxlIntf* self(motor_drive_intf_t* p){ return (DxlIntf*)p; }

static int write_u1(DynamixelBus* b, uint8_t id, uint16_t a, uint8_t v)
{ return b->write(id, a, 1, &v); }

static int write_u4(DynamixelBus* b, uint8_t id, uint16_t a, uint32_t v)
{
    uint8_t buf[4] = { uint8_t(v), uint8_t(v>>8), uint8_t(v>>16), uint8_t(v>>24) };
    return b->write(id, a, 4, buf);
}

static int32_t op_free(motor_drive_intf_t* p){ delete self(p); return 0; }

static int32_t op_enable(motor_drive_intf_t* p)
{
    return write_u1(self(p)->bus, self(p)->id, kAddrTorqueEn, 1);
}
static int32_t op_disable(motor_drive_intf_t* p)
{
    return write_u1(self(p)->bus, self(p)->id, kAddrTorqueEn, 0);
}
static int32_t op_fault_reset(motor_drive_intf_t* p)
{
    /* Dynamixel has no explicit reset instruction — the manual says
     * toggle torque off / on clears a fault after the root cause is
     * gone. */
    if (op_disable(p) != 0){ return -1; }
    return op_enable(p);
}

static int32_t op_set_mode(motor_drive_intf_t* p, motor_intf_mode_t m)
{
    uint8_t v;
    switch (m){
    case MOTOR_INTF_MODE_TORQUE:   v = kModeCurrent;  break;
    case MOTOR_INTF_MODE_VELOCITY: v = kModeVelocity; break;
    case MOTOR_INTF_MODE_POSITION: v = kModePosition; break;
    default:                       v = kModePosition; break;
    }
    /* Op-mode is RAM-only when torque is on — disable first, write
     * mode, re-enable in the caller's next bringup/enable. */
    write_u1(self(p)->bus, self(p)->id, kAddrTorqueEn, 0);
    const int rc = write_u1(self(p)->bus, self(p)->id, kAddrOpMode, v);
    if (rc == 0){ self(p)->last_mode = m; }
    return rc;
}
static int32_t op_get_mode(motor_drive_intf_t* p, motor_intf_mode_t* out)
{
    if (!out){ return -1; }
    uint8_t v = 0;
    const int rc = self(p)->bus->read(self(p)->id, kAddrOpMode, 1, &v);
    if (rc != 0){ return rc; }
    switch (v){
    case kModeCurrent:  *out = MOTOR_INTF_MODE_TORQUE;   break;
    case kModeVelocity: *out = MOTOR_INTF_MODE_VELOCITY; break;
    case kModePosition:
    case kModeExtPos:   *out = MOTOR_INTF_MODE_POSITION; break;
    default:            *out = MOTOR_INTF_MODE_NONE;     break;
    }
    return 0;
}

static int32_t op_bringup(motor_drive_intf_t* p, uint32_t /*timeout_ms*/)
{
    /* Ping → mode → torque-on. */
    if (self(p)->bus->ping(self(p)->id) != 0){ return -1; }
    op_set_mode(p, self(p)->bringup_mode);
    return op_enable(p);
}

static int32_t op_set_position(motor_drive_intf_t* p, float rad)
{
    const int32_t raw = int32_t(std::round(rad / kRadPerCount));
    return write_u4(self(p)->bus, self(p)->id, kAddrGoalPos, uint32_t(raw));
}
static int32_t op_set_velocity(motor_drive_intf_t* p, float rps)
{
    const int32_t raw = int32_t(std::round(rps / kRadPerSecPerUnit));
    return write_u4(self(p)->bus, self(p)->id, kAddrGoalVel, uint32_t(raw));
}
static int32_t op_set_torque(motor_drive_intf_t* p, float nm)
{
    /* XL-330 uses 1 mA/unit, XM-430 uses 2.69 mA/unit. A_per_Nm lets
     * the caller unit-match; 0 means "user passes A directly". */
    float mA = (self(p)->A_per_Nm > 0.0f) ? nm * self(p)->A_per_Nm * 1000.0f
                                          : nm * 1000.0f;
    int16_t raw = int16_t(std::round(mA));
    uint8_t buf[2] = { uint8_t(raw), uint8_t(raw >> 8) };
    return self(p)->bus->write(self(p)->id, kAddrGoalCurrent, 2, buf);
}

static int32_t op_get_position(motor_drive_intf_t* p, float* out)
{
    if (!out){ return -1; }
    uint8_t buf[4];
    const int rc = self(p)->bus->read(self(p)->id, kAddrPresentPos, 4, buf);
    if (rc != 0){ return rc; }
    const int32_t raw = int32_t(buf[0]) | (int32_t(buf[1]) << 8) |
                        (int32_t(buf[2]) << 16) | (int32_t(buf[3]) << 24);
    *out = float(raw * kRadPerCount);
    return 0;
}
static int32_t op_get_velocity(motor_drive_intf_t* p, float* out)
{
    if (!out){ return -1; }
    uint8_t buf[4];
    const int rc = self(p)->bus->read(self(p)->id, kAddrPresentVel, 4, buf);
    if (rc != 0){ return rc; }
    const int32_t raw = int32_t(buf[0]) | (int32_t(buf[1]) << 8) |
                        (int32_t(buf[2]) << 16) | (int32_t(buf[3]) << 24);
    *out = float(raw * kRadPerSecPerUnit);
    return 0;
}
static int32_t op_get_current(motor_drive_intf_t* p, float* A)
{
    if (!A){ return -1; }
    uint8_t buf[2];
    const int rc = self(p)->bus->read(self(p)->id, kAddrPresentCurrent, 2, buf);
    if (rc != 0){ return rc; }
    const int16_t raw = int16_t(buf[0] | (buf[1] << 8));
    *A = float(raw) * 0.001f;      /* default 1 mA/unit (XL-330) */
    return 0;
}
static int32_t op_get_torque(motor_drive_intf_t* p, float* Nm)
{
    if (!Nm){ return -1; }
    float A = 0.0f;
    const int rc = op_get_current(p, &A);
    if (rc != 0){ return rc; }
    *Nm = (self(p)->A_per_Nm > 0.0f) ? (A / self(p)->A_per_Nm) : A;
    return 0;
}
static int32_t op_get_temperature(motor_drive_intf_t* p, float* degC)
{
    if (!degC){ return -1; }
    uint8_t v = 0;
    const int rc = self(p)->bus->read(self(p)->id, kAddrPresentTemp, 1, &v);
    if (rc != 0){ return rc; }
    *degC = float(v);
    return 0;
}

static int32_t op_get_state(motor_drive_intf_t* p, motor_intf_state_t* out)
{
    if (!out){ return -1; }
    uint8_t tq = 0, hw = 0;
    const int rc1 = self(p)->bus->read(self(p)->id, kAddrTorqueEn, 1, &tq);
    if (rc1 != 0){ *out = MOTOR_INTF_STATE_OFFLINE; return rc1; }
    (void)self(p)->bus->read(self(p)->id, kAddrHwError, 1, &hw);
    if (hw != 0){ *out = MOTOR_INTF_STATE_FAULT; return 0; }
    *out = tq ? MOTOR_INTF_STATE_ENABLED : MOTOR_INTF_STATE_DISABLED;
    return 0;
}

static int32_t op_set_id(motor_drive_intf_t* p, uint8_t newId)
{
    /* Address 7 in the control table is the ID byte; requires torque
     * off. We preserve the previous id for the adapter after the write
     * succeeds. */
    write_u1(self(p)->bus, self(p)->id, kAddrTorqueEn, 0);
    uint8_t v = newId;
    const int rc = self(p)->bus->write(self(p)->id, 7, 1, &v);
    if (rc == 0){
        self(p)->id = newId;
        std::snprintf(self(p)->name, sizeof(self(p)->name),
                      "dxl-%u", unsigned(newId));
    }
    return rc;
}
static int32_t op_get_id(motor_drive_intf_t* p, uint8_t* out)
{
    if (!out){ return -1; }
    *out = self(p)->id;
    return 0;
}
static const char* op_name(motor_drive_intf_t* p){ return self(p)->name; }

static const motor_drive_intf_proc_t kDxlProc = {
    /* .free            */ op_free,
    /* .enable          */ op_enable,
    /* .disable         */ op_disable,
    /* .fault_reset     */ op_fault_reset,
    /* .bringup         */ op_bringup,
    /* .set_mode        */ op_set_mode,
    /* .get_mode        */ op_get_mode,
    /* .set_torque      */ op_set_torque,
    /* .set_velocity    */ op_set_velocity,
    /* .set_position    */ op_set_position,
    /* .set_torque_limit*/ nullptr,
    /* .set_vel_limit   */ nullptr,
    /* .set_pos_limit   */ nullptr,
    /* .set_gain        */ nullptr,
    /* .get_gain        */ nullptr,
    /* .get_position    */ op_get_position,
    /* .get_velocity    */ op_get_velocity,
    /* .get_torque      */ op_get_torque,
    /* .get_current     */ op_get_current,
    /* .get_temperature */ op_get_temperature,
    /* .get_state       */ op_get_state,
    /* .set_id          */ op_set_id,
    /* .get_id          */ op_get_id,
    /* .name            */ op_name,
};

}  // namespace


motor_drive_intf_t* makeDynamixelIntf(const DynamixelIntfCfg& cfg)
{
    if (!cfg.bus){ return nullptr; }
    auto* x = new DxlIntf();
    x->base.proc    = &kDxlProc;
    x->bus          = cfg.bus;
    x->id           = cfg.id;
    x->A_per_Nm     = cfg.A_per_Nm;
    x->bringup_mode = cfg.bringup_mode;
    x->last_mode    = MOTOR_INTF_MODE_NONE;
    if (cfg.name && *cfg.name){
        std::snprintf(x->name, sizeof(x->name), "%s", cfg.name);
    } else {
        std::snprintf(x->name, sizeof(x->name), "dxl-%u", unsigned(cfg.id));
    }
    return &x->base;
}

}  // namespace vrmc
