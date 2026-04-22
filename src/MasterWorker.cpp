#include "MasterWorker.hpp"

#include <QDateTime>
#include <QMutexLocker>
#include <QSet>
#include <QTimer>
#include <cmath>

extern "C" {
#include "master_mgr.h"
#include "motor_drive_interface.h"
}

namespace vrmc {

static master_mgr_target_t toMgrTarget(TargetKind k)
{
    switch (k){
    case TargetKind::Position: return MASTER_MGR_TARGET_POSITION;
    case TargetKind::Velocity: return MASTER_MGR_TARGET_VELOCITY;
    case TargetKind::Torque:   return MASTER_MGR_TARGET_TORQUE;
    }
    return MASTER_MGR_TARGET_POSITION;
}

static motor_intf_mode_t toIntfMode(Mode m)
{
    switch (m){
    case Mode::None:     return MOTOR_INTF_MODE_NONE;
    case Mode::Torque:   return MOTOR_INTF_MODE_TORQUE;
    case Mode::Velocity: return MOTOR_INTF_MODE_VELOCITY;
    case Mode::Position: return MOTOR_INTF_MODE_POSITION;
    }
    return MOTOR_INTF_MODE_NONE;
}

static motor_intf_loop_t toIntfLoop(Loop l)
{
    switch (l){
    case Loop::Current:  return MOTOR_INTF_LOOP_CURRENT;
    case Loop::Velocity: return MOTOR_INTF_LOOP_VELOCITY;
    case Loop::Position: return MOTOR_INTF_LOOP_POSITION;
    }
    return MOTOR_INTF_LOOP_CURRENT;
}

MasterWorker::MasterWorker(QObject* parent) : QObject(parent)
{
    m_tick = new QTimer(this);
    m_tick->setTimerType(Qt::PreciseTimer);
    connect(m_tick, &QTimer::timeout, this, &MasterWorker::onTick);

    m_genTimer = new QTimer(this);
    m_genTimer->setTimerType(Qt::PreciseTimer);
    connect(m_genTimer, &QTimer::timeout, this, &MasterWorker::onGenTick);
}

MasterWorker::~MasterWorker() { teardown(); }

void MasterWorker::connectCan(const CanConfig& cfg)
{
    QMutexLocker lock(&m_mutex);
    if (m_mgr){
        emit error(QStringLiteral("already connected"));
        return;
    }
    m_mgr = master_mgr_create();
    if (!m_mgr){
        emit error(QStringLiteral("master_mgr_create failed"));
        return;
    }

    QString err;
    if (!m_can.open(m_mgr, cfg, &err)){
        master_mgr_destroy(m_mgr);
        m_mgr = nullptr;
        emit error(err);
        return;
    }

    m_tick->start(1000 / std::max(1, m_hz));
    emit info(QStringLiteral("connected: %1 slave(s)").arg(cfg.count));
    emit connected(cfg.count);
}

void MasterWorker::disconnect_()
{
    teardown();
    emit disconnected();
}

void MasterWorker::teardown()
{
    QMutexLocker lock(&m_mutex);
    if (m_tick){ m_tick->stop(); }
    m_can.close();
    if (m_mgr){
        master_mgr_destroy(m_mgr);
        m_mgr = nullptr;
    }
}

void MasterWorker::setRefreshHz(int hz)
{
    m_hz = std::max(1, hz);
    if (m_tick && m_tick->isActive()){
        m_tick->start(1000 / m_hz);
    }
}

/* CiA 402 wire conversions for the motor_drive_cia402 simulator. The
 * sim uses a 14-bit encoder (16384 counts/rev) and reports torque as
 * per-mille of @c RATED_TORQUE_NM = 0.5 Nm. Move into MotorParams once
 * the diagnostic supports per-slave conversion factors. */
static constexpr double kCountsPerRev = 16384.0;
static constexpr double kRadPerCount  = (2.0 * M_PI) / kCountsPerRev;
static constexpr double kRatedTorqueNm = 0.5;

/* CiA 402 statusword → motor_intf_state_t. Lets the table's state
 * column stay live off PDO alone; avoids a per-tick SDO read of state. */
static int stateFromStatusword(uint16_t sw)
{
    /* Bit masks per CiA 402 §6.3. Only the low byte is state. */
    if (sw & (1u << 3)){ return 7; }                         /* FAULT */
    const bool rsw = (sw & 0x01);   // ready to switch on
    const bool son = (sw & 0x02);   // switched on
    const bool ope = (sw & 0x04);   // operation enabled
    const bool qsa = !(sw & 0x20);  // quick stop active (bit 5 inverted)
    const bool sod = (sw & 0x40);   // switch on disabled
    if (ope && son && rsw){ return 4; }                      /* OPERATION_ENABLED */
    if (son && rsw){       return 3; }                       /* SWITCHED_ON */
    if (rsw){              return 2; }                       /* READY_TO_SWITCH_ON */
    if (sod){              return 1; }                       /* SWITCH_ON_DISABLED */
    if (qsa){              return 5; }                       /* QUICK_STOP_ACTIVE */
    return 0;                                                 /* NOT_READY */
}

void MasterWorker::onTick()
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }

    m_can.pump();

    /* SDO refresh strategy: if PDO is alive, skip the per-tick SDO poll
     * entirely — telemetry comes from cached TPDO frames. Fire a
     * housekeeping refresh every second so name / id / fallback state
     * stay current for slaves that briefly drop PDO. Without PDO
     * (Feetech, Dynamixel, ZLG today) keep the original per-tick refresh. */
    const bool pdo = m_can.hasPdo();
    ++m_refreshTick;
    const int houseKeepEvery = std::max(1, m_hz);  /* 1 Hz */
    if (!pdo || m_refreshTick >= houseKeepEvery){
        master_mgr_refresh(m_mgr);
        m_refreshTick = 0;
    }

    QVector<SlaveSnapshot> out;
    const uint32_t n = master_mgr_slave_count(m_mgr);
    out.reserve(n);
    for (uint32_t i = 0; i < n; ++i){
        master_slave_info_t mi{};   /* not "info" — shadows signal */
        if (master_mgr_get_info(m_mgr, i, &mi) != 0){ continue; }
        SlaveSnapshot s;
        s.idx      = static_cast<int>(i);
        s.id       = mi.id;
        s.state    = mi.state;
        s.online   = mi.online;
        /* SDO baseline (motor_drive_intf SI conversion). PDO override
         * below if the slave has been emitting TPDO1. */
        s.position = mi.position;
        s.velocity = mi.velocity;
        s.torque   = mi.torque;
        s.name     = QString::fromUtf8(mi.name ? mi.name : "");

        CanBackend::PdoFrame pf;
        if (m_can.getPdo(i, &pf)){
            s.pdoFresh   = true;
            s.statusword = pf.statusword;
            s.errorCode  = pf.error_code;
            s.pdoRxCount = pf.rx_count;
            /* Replace SDO actuals with the freshly-decoded PDO frame. */
            s.position = static_cast<float>(pf.position * kRadPerCount);
            s.velocity = static_cast<float>(pf.velocity * kRadPerCount);
            s.torque   = static_cast<float>(pf.torque  / 1000.0 * kRatedTorqueNm);
            /* PDO is our freshness proof; override state/online too so
             * the table doesn't flicker between the slow SDO refresh
             * (1 Hz) and the PDO tick (100 Hz). */
            s.state  = stateFromStatusword(s.statusword);
            s.online = true;
            /* Announce once per slave so the log shows PDO is alive. */
            static QSet<int> announced;
            if (!announced.contains(s.idx)){
                announced.insert(s.idx);
                /* Local @c info shadows MasterWorker::info; use Q_EMIT. */
                Q_EMIT info(QStringLiteral("PDO online: slave %1 - rx=%2, "
                                           "statusword=0x%3")
                                .arg(s.idx)
                                .arg(qulonglong(s.pdoRxCount))
                                .arg(s.statusword, 4, 16, QChar('0')));
            }
        }

        /* Extended channels. If the backend supports the intf getter use it;
         * otherwise fall back to a plausibly-shaped derivation so the
         * diagnostic has useful content against a minimal simulator. */
        auto* slave = master_mgr_get_slave(m_mgr, i);
        float current = 0.0f;
        if (!slave || motor_drive_intf_get_current(slave, &current) != 0){
            /* Fake: assume Kt = 0.05 Nm/A -> current = torque / 0.05. */
            current = s.torque / 0.05f;
        }
        s.current = current;

        float temp = 0.0f;
        if (!slave || motor_drive_intf_get_temperature(slave, &temp) != 0){
            /* Fake: 25 °C ambient + rise proportional to |v| and |T|. */
            temp = 25.0f + std::abs(s.velocity) * 1.2f
                         + std::abs(s.torque)   * 20.0f;
        }
        s.temperature = temp;

        /* Commanded setpoints + tracking error (from the worker's own cache). */
        const auto it = m_cmdCache.constFind(s.idx);
        if (it != m_cmdCache.cend()){
            s.cmdPosition = it->pos;
            s.cmdVelocity = it->vel;
            s.cmdTorque   = it->trq;
            switch (it->last){
            case TargetKind::Position: s.trackingError = it->pos - s.position; break;
            case TargetKind::Velocity: s.trackingError = it->vel - s.velocity; break;
            case TargetKind::Torque:   s.trackingError = it->trq - s.torque;   break;
            }
        }
        out.push_back(s);
    }
    lock.unlock();
    emit snapshots(std::move(out));
}

void MasterWorker::refreshOnce() { onTick(); }

void MasterWorker::bringupOne(int idx, uint32_t timeoutMs)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_bringup_one(m_mgr, idx, timeoutMs) != 0){
        emit error(QStringLiteral("bringup slave %1 failed").arg(idx));
    } else {
        emit info(QStringLiteral("bringup slave %1 ok").arg(idx));
    }
}

void MasterWorker::enableOne(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_enable_one(m_mgr, idx) != 0){
        emit error(QStringLiteral("enable slave %1 failed").arg(idx));
    }
}

void MasterWorker::disableOne(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_disable_one(m_mgr, idx) != 0){
        emit error(QStringLiteral("disable slave %1 failed").arg(idx));
    }
}

void MasterWorker::faultReset(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_fault_reset_one(m_mgr, idx) != 0){
        emit error(QStringLiteral("fault reset slave %1 failed").arg(idx));
    }
}

void MasterWorker::setMode(int idx, Mode mode)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_set_mode_one(m_mgr, idx, toIntfMode(mode)) != 0){
        emit error(QStringLiteral("set mode slave %1 failed").arg(idx));
    }
}

void MasterWorker::setTarget(int idx, TargetKind which, float value)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_set_target_one(m_mgr, idx, toMgrTarget(which), value) != 0){
        emit error(QStringLiteral("set target slave %1 failed").arg(idx));
        return;
    }
    auto& c = m_cmdCache[idx];
    c.last = which;
    switch (which){
    case TargetKind::Position: c.pos = value; break;
    case TargetKind::Velocity: c.vel = value; break;
    case TargetKind::Torque:   c.trq = value; break;
    }
}

void MasterWorker::setId(int idx, uint8_t newId)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    if (master_mgr_set_id_one(m_mgr, idx, newId) != 0){
        emit error(QStringLiteral("set id slave %1 failed").arg(idx));
    }
}

/* --- E-stop ---------------------------------------------------------- */

void MasterWorker::disableAll()
{
    QMutexLocker lock(&m_mutex);
    /* Also stop any running signal generator so we don't immediately
     * re-enable and start driving the bus. */
    if (m_genTimer && m_genTimer->isActive()){
        m_genTimer->stop();
        const int genIdx = m_genIdx;
        m_genIdx = -1;
        if (genIdx >= 0){ emit generatorStopped(genIdx); }
    }
    if (!m_mgr){ return; }
    const int n = master_mgr_disable_all(m_mgr);
    emit info(QStringLiteral("E-STOP: disabled %1 slave(s)").arg(n));
}

/* --- Gain r/w -------------------------------------------------------- */

void MasterWorker::readGain(int idx, Loop loop)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    auto* slave = master_mgr_get_slave(m_mgr, idx);
    if (!slave){
        emit gainRead(idx, loop, 0.0f, 0.0f, false);
        return;
    }
    float32_t kp = 0.0f, ki = 0.0f;
    const int rc = motor_drive_intf_get_gain(slave, toIntfLoop(loop), &kp, &ki);
    emit gainRead(idx, loop, kp, ki, rc == 0);
    if (rc != 0){
        emit error(QStringLiteral("read gain slave %1 loop %2 failed (rc=%3)")
                       .arg(idx).arg(int(loop)).arg(rc));
    }
}

void MasterWorker::writeGain(int idx, Loop loop, float kp, float ki)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    auto* slave = master_mgr_get_slave(m_mgr, idx);
    if (!slave){ return; }
    const int rc = motor_drive_intf_set_gain(slave, toIntfLoop(loop), kp, ki);
    if (rc != 0){
        emit error(QStringLiteral("write gain slave %1 loop %2 failed (rc=%3)")
                       .arg(idx).arg(int(loop)).arg(rc));
    } else {
        emit info(QStringLiteral("slave %1 loop %2 gain = kp:%3 ki:%4")
                      .arg(idx).arg(int(loop))
                      .arg(kp, 0, 'g', 4).arg(ki, 0, 'g', 4));
    }
}

/* --- Signal generator ----------------------------------------------- */

void MasterWorker::startGenerator(int idx, GenCfg cfg)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit error(QStringLiteral("generator: not connected"));
        return;
    }
    if (m_genTimer->isActive()){
        m_genTimer->stop();
        const int prev = m_genIdx;
        m_genIdx = -1;
        if (prev >= 0){ emit generatorStopped(prev); }
    }
    m_genCfg = cfg;
    m_genIdx = idx;
    m_genT0  = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    const int periodMs = std::max(1, 1000 / std::max(1, cfg.rateHz));
    m_genTimer->start(periodMs);
    emit generatorStarted(idx);
    emit info(QStringLiteral("generator: slave %1 shape %2 amp %3 f %4 Hz for %5 s")
                  .arg(idx).arg(int(cfg.shape))
                  .arg(cfg.amplitude, 0, 'f', 3)
                  .arg(cfg.frequency, 0, 'f', 3)
                  .arg(cfg.durationSec, 0, 'f', 1));
}

void MasterWorker::stopGenerator(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_genTimer->isActive()){ return; }
    if (idx >= 0 && idx != m_genIdx){ return; }
    m_genTimer->stop();
    const int prev = m_genIdx;
    m_genIdx = -1;
    if (prev >= 0){ emit generatorStopped(prev); }
}

/* --- Drive-parameter batch I/O ------------------------------------- */

namespace {

/* One OD sub-object to pull / push. The SDO stack serialises in little-
 * endian, so we just memcpy into/out of the struct fields directly. */
struct DriveField {
    uint16_t idx;
    uint8_t  sub;
    uint8_t  len;
    void*    ptr;
};

template <typename T>
static DriveField fld(uint16_t i, uint8_t s, T& v)
{
    return { i, s, uint8_t(sizeof(T)), &v };
}

static QVector<DriveField> driveFields(DriveConfig& c)
{
    return {
        fld(0x6098, 0, c.homing_method),
        fld(0x607C, 0, c.home_offset),
        fld(0x6099, 1, c.homing_speed_fast),
        fld(0x6099, 2, c.homing_speed_slow),
        fld(0x609A, 0, c.homing_accel),

        fld(0x6081, 0, c.profile_velocity),
        fld(0x6083, 0, c.profile_accel),
        fld(0x6084, 0, c.profile_decel),
        fld(0x6085, 0, c.quickstop_decel),

        fld(0x6065, 0, c.following_error),
        fld(0x6066, 0, c.following_error_ms),
        fld(0x607D, 1, c.pos_limit_min),
        fld(0x607D, 2, c.pos_limit_max),
        fld(0x6080, 0, c.max_motor_speed),
        fld(0x6072, 0, c.max_torque),

        fld(0x608F, 1, c.enc_resolution),
        fld(0x6091, 1, c.gear_num),
        fld(0x6091, 2, c.gear_den),
        fld(0x6092, 1, c.feed_const_num),
        fld(0x6092, 2, c.feed_const_den),
    };
}

}  // namespace

void MasterWorker::readDriveConfig(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit driveConfigRead(idx, DriveConfig{}, false,
                             QStringLiteral("not connected"));
        return;
    }
    DriveConfig c{};
    auto fields = driveFields(c);
    /* Fast-path abort: if the first three reads all fail, odds are the
     * slave is unreachable (wrong id, bus down) or the firmware doesn't
     * expose any of these OD entries. Bail early so we don't block the
     * UI for 20 × 100 ms. */
    constexpr int kEarlyBailAfter = 3;
    int okCount = 0;
    int failCount = 0;
    QStringList failures;
    bool bailed = false;
    for (int i = 0; i < int(fields.size()); ++i){
        const auto& f = fields[i];
        QString err;
        const int rc = m_can.readSdo(idx, f.idx, f.sub, f.ptr, f.len, &err);
        if (rc == 0){
            ++okCount;
        } else {
            ++failCount;
            failures << err;
            if (okCount == 0 && failCount >= kEarlyBailAfter){
                bailed = true;
                break;
            }
        }
    }

    const bool anyOk = (okCount > 0);
    QString msg;
    if (bailed){
        msg = QStringLiteral(
            "Drive did not respond to the first %1 reads. Check:\n"
            "  • slave is online (state column)\n"
            "  • node id matches the Connect-dialog range\n"
            "  • firmware exposes CiA 402 config objects "
            "(the motor_drive_cia402 simulator does not — it only "
            "implements statusword, controlword, mode, and actuals)"
        ).arg(kEarlyBailAfter);
    } else if (!failures.isEmpty()){
        msg = QStringLiteral("%1 of %2 fields read. Missing:\n  %3")
                  .arg(okCount).arg(fields.size())
                  .arg(failures.join(QStringLiteral("\n  ")));
    }

    if (anyOk){
        emit info(QStringLiteral("drive config: slave %1 - %2/%3 fields read")
                      .arg(idx).arg(okCount).arg(fields.size()));
    } else {
        emit error(QStringLiteral("drive config read: slave %1 - no fields "
                                  "readable (%2)").arg(idx)
                       .arg(bailed ? QStringLiteral("early bail")
                                   : QStringLiteral("all aborted")));
    }
    emit driveConfigRead(idx, c, anyOk, msg);
}

void MasterWorker::writeDriveConfig(int idx, DriveConfig cfg)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit driveConfigWritten(idx, false,
                                QStringLiteral("not connected"));
        return;
    }
    auto fields = driveFields(cfg);
    QStringList failures;
    for (const auto& f : fields){
        QString err;
        const int rc = m_can.writeSdo(idx, f.idx, f.sub, f.ptr, f.len, &err);
        if (rc != 0){ failures << err; }
    }
    if (failures.isEmpty()){
        emit info(QStringLiteral("drive config: slave %1 written (%2 fields)")
                      .arg(idx).arg(fields.size()));
        emit driveConfigWritten(idx, true, QString());
    } else {
        const QString msg = failures.join(QStringLiteral("\n"));
        emit error(QStringLiteral("drive config write: slave %1 — %2 of %3 "
                                  "fields failed")
                       .arg(idx)
                       .arg(failures.size())
                       .arg(fields.size()));
        emit driveConfigWritten(idx, false, msg);
    }
}

/* --- Commissioning one-shots --------------------------------------- */

void MasterWorker::startHoming(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit error(QStringLiteral("start-homing: not connected"));
        return;
    }
    /* CiA 402 homing sequence, assuming the drive is already in
     * Operation Enabled (user hit Enable first):
     *   1) 0x6060 = 6              — mode of operation = Homing
     *   2) 0x6040 = 0x000F         — controlword: enable op, bit 4 low
     *   3) 0x6040 = 0x001F         — rising edge on bit 4 = start homing
     * Progress is visible on statusword bits 12 (attained) / 13 (error),
     * already surfaced through the live SlaveSnapshot. */
    uint8_t  mode = 6;
    uint16_t cw0  = 0x000F;
    uint16_t cw1  = 0x001F;
    QString  err;
    if (m_can.writeSdo(idx, 0x6060, 0, &mode, 1, &err) != 0 ||
        m_can.writeSdo(idx, 0x6040, 0, &cw0,  2, &err) != 0 ||
        m_can.writeSdo(idx, 0x6040, 0, &cw1,  2, &err) != 0){
        emit error(QStringLiteral("start-homing: slave %1 — %2")
                       .arg(idx).arg(err));
        return;
    }
    emit info(QStringLiteral("homing started on slave %1 (watch "
                             "statusword bit 12 / 13)").arg(idx));
}

void MasterWorker::zeroEncoderHere(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit calibrationDone(idx, QStringLiteral("encoder-zero"), 0, false,
                             QStringLiteral("not connected"));
        return;
    }
    int32_t pos = 0;
    QString err;
    if (m_can.readSdo (idx, 0x6064, 0, &pos, 4, &err) != 0 ||
        m_can.writeSdo(idx, 0x607C, 0, &pos, 4, &err) != 0){
        emit error(QStringLiteral("encoder zero: slave %1 — %2")
                       .arg(idx).arg(err));
        emit calibrationDone(idx, QStringLiteral("encoder-zero"), pos,
                             false, err);
        return;
    }
    emit info(QStringLiteral("encoder zeroed: slave %1 home_offset = %2 cnt")
                  .arg(idx).arg(pos));
    emit calibrationDone(idx, QStringLiteral("encoder-zero"), pos, true,
                         QString());
}

void MasterWorker::zeroTorqueHere(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit calibrationDone(idx, QStringLiteral("torque-zero"), 0, false,
                             QStringLiteral("not connected"));
        return;
    }
    int16_t t = 0;
    QString err;
    if (m_can.readSdo (idx, 0x6077, 0, &t, 2, &err) != 0 ||
        m_can.writeSdo(idx, 0x60B2, 0, &t, 2, &err) != 0){
        emit error(QStringLiteral("torque zero: slave %1 — %2")
                       .arg(idx).arg(err));
        emit calibrationDone(idx, QStringLiteral("torque-zero"),
                             int64_t(t), false, err);
        return;
    }
    emit info(QStringLiteral("torque offset set: slave %1 = %2 ‰ rated")
                  .arg(idx).arg(t));
    emit calibrationDone(idx, QStringLiteral("torque-zero"), int64_t(t),
                         true, QString());
}

/* --- PDO mapping ---------------------------------------------------- */

void MasterWorker::applyPdoMapping(int idx, bool isTpdo,
                                   QVector<CanBackend::PdoMapEntry> entries)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit pdoMappingApplied(idx, isTpdo, false,
                               QStringLiteral("not connected"));
        return;
    }
    std::vector<CanBackend::PdoMapEntry> v(entries.begin(), entries.end());
    QString err;
    const int rc = m_can.writePdoMapping(idx, isTpdo, v, &err);
    const bool ok = (rc == 0);
    const QString label = isTpdo ? QStringLiteral("TPDO1")
                                 : QStringLiteral("RPDO1");
    if (ok){
        emit info(QStringLiteral("%1 mapping: slave %2 applied (%3 entries)")
                      .arg(label).arg(idx).arg(entries.size()));
    } else {
        emit error(QStringLiteral("%1 mapping: slave %2 failed — %3")
                       .arg(label).arg(idx).arg(err));
    }
    emit pdoMappingApplied(idx, isTpdo, ok, err);
}

void MasterWorker::onGenTick()
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr || m_genIdx < 0){
        m_genTimer->stop();
        return;
    }
    const double now = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    const double t   = now - m_genT0;

    if (m_genCfg.durationSec > 0.0f && t >= m_genCfg.durationSec){
        m_genTimer->stop();
        const int prev = m_genIdx;
        m_genIdx = -1;
        emit info(QStringLiteral("generator: slave %1 done").arg(prev));
        if (prev >= 0){ emit generatorStopped(prev); }
        return;
    }

    float value = m_genCfg.offset;
    const float A  = m_genCfg.amplitude;
    const float f0 = m_genCfg.frequency;
    switch (m_genCfg.shape){
    case GenCfg::Constant:
        value += A;
        break;
    case GenCfg::Step:
        value += (t >= 0.0 ? A : 0.0f);
        break;
    case GenCfg::Ramp: {
        const float slope = (m_genCfg.durationSec > 0.0f)
                                ? (A / m_genCfg.durationSec)
                                : A;
        value += static_cast<float>(slope * t);
        break;
    }
    case GenCfg::Sine:
        value += A * std::sin(2.0 * M_PI * f0 * t);
        break;
    case GenCfg::Chirp: {
        const float f1 = m_genCfg.frequencyEnd;
        const float T  = (m_genCfg.durationSec > 0.0f)
                             ? m_genCfg.durationSec : 1.0f;
        const double phi = 2.0 * M_PI * (f0 * t + (f1 - f0) * t * t / (2.0 * T));
        value += A * std::sin(phi);
        break;
    }
    }
    master_mgr_set_target_one(m_mgr, m_genIdx,
                              toMgrTarget(m_genCfg.target), value);

    auto& c = m_cmdCache[m_genIdx];
    c.last = m_genCfg.target;
    switch (m_genCfg.target){
    case TargetKind::Position: c.pos = value; break;
    case TargetKind::Velocity: c.vel = value; break;
    case TargetKind::Torque:   c.trq = value; break;
    }
}

}  // namespace vrmc
