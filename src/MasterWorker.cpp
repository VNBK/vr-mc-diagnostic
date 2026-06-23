#include "MasterWorker.hpp"

#include <QDateTime>
#include <QMutexLocker>
#include <QSet>
#include <QTimer>
#include <algorithm>
#include <cmath>

extern "C" {
#include "master_mgr.h"
#include "motor_drive_interface.h"
#include "cia402/cia402_utils.h"
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

    m_cycle = new QTimer(this);
    m_cycle->setTimerType(Qt::PreciseTimer);
    connect(m_cycle, &QTimer::timeout, this, &MasterWorker::onCycleTick);

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
    emit info(QStringLiteral("Number slave is scan: %1 slaves").arg(cfg.count));
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

    m_tick ->start(1000 / std::max(1, m_hz));
    m_cycle->start(1000 / std::max(1, m_cycleHz));
    emit info(QStringLiteral("connected: %1 slave(s), PDO cycle %2 Hz")
                  .arg(cfg.count).arg(m_cycleHz));
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
    if (m_tick) { m_tick ->stop(); }
    if (m_cycle){ m_cycle->stop(); }
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

/* CiA 402 wire scaling. Now driven by the active motor profile via
 * MasterWorker::setScaling (m_countsPerRev / m_ratedNm); these defaults
 * only apply until a profile loads. Used with the SDK's cia402_utils
 * helpers (cia402_inc_to_rad, cia402_nm_to_milli_rated…). */

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

void MasterWorker::onCycleTick()
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Drain transport RX (paired TPDOs land in the per-slot cache) and
     * send one RPDO per registered slot using its cached controlword +
     * targets. Mirrors motor_drive_master.c's cycle_step. Runs at
     * m_cycleHz (1 kHz default) — independent of UI refresh. */
    m_can.pump();
    m_can.cycleStep();
}

void MasterWorker::onTick()
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }

    /* UI refresh path only — RPDO sending + transport pumping live in
     * onCycleTick, which runs at m_cycleHz (faster than m_hz). Pumping
     * here too would just contend for the transport without making
     * additional progress. SDO ops have their own pump callback
     * (pumpStub) so they don't starve between cycle ticks either. */

    /* SDO refresh strategy: if PDO is alive, skip the per-tick SDO poll
     * entirely — telemetry comes from cached TPDO frames. Fire a
     * housekeeping refresh every second so name / id / fallback state
     * stay current for slaves that briefly drop PDO. Without PDO
     * (Feetech, Dynamixel) keep the original per-tick refresh. */
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
            /* Replace SDO actuals with the freshly-decoded PDO frame.
             * Conversions via the SDK's cia402_utils inlines. */
            s.position = cia402_inc_to_rad(pf.position, m_countsPerRev);
            s.velocity = cia402_inc_s_to_rad_s(pf.velocity, m_countsPerRev);
            s.torque   = cia402_milli_rated_to_nm(pf.torque, m_ratedNm);
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
        if (s.pdoFresh){
            /* Current actual now streams in the TPDO (0x6078, per-mille of
             * rated current) — prefer it over an SDO read. */
            current = cia402_milli_rated_to_amps(pf.current, m_ratedA);
        } else if (!slave || motor_drive_intf_get_current(slave, &current) != 0){
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

/* CiA-402 controlword bit patterns. See @ref cia402_state_machine in
 * vr-mc-sdk/src/comm/cia402/cia402_master.h for the full state table. */
static constexpr uint16_t kCwEnableOperation  = 0x000F;   /* Switch On + Enable Voltage + Quick Stop + Enable Op */
static constexpr uint16_t kCwShutdown         = 0x0006;   /* Disable on next walk     */
static constexpr uint16_t kCwQuickStop        = 0x0002;   /* Controlled decel + brake */
static constexpr uint16_t kCwFaultReset       = 0x0080;   /* Rising edge of bit 7     */

void MasterWorker::bringupOne(int idx, uint32_t timeoutMs)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Park the cached cycle controlword at 0x0000 (Disable Voltage) for
     * the SDO bringup walk so the cycle's RPDOs don't fight the SDO
     * writes done by cia402_master_bringup. After the walk completes,
     * latch to 0x000F so the cycle holds OPERATION_ENABLED. */
    m_can.setCycleControlword(static_cast<uint32_t>(idx), 0);
    if (master_mgr_bringup_one(m_mgr, idx, timeoutMs) != 0){
        emit error(QStringLiteral("bringup slave %1 failed").arg(idx));
        emit controlwordCached(idx, 0);
    } else {
        emit info(QStringLiteral("bringup slave %1 ok").arg(idx));
        m_can.setCycleControlword(static_cast<uint32_t>(idx), kCwEnableOperation);
        emit controlwordCached(idx, kCwEnableOperation);

        /* Pin the telemetry current scale to the drive's ACTUAL rated current
         * (0x6075, mA). 0x6078 streams per-mille of THIS value, so decoding it
         * with a stale profile rated mis-scales the displayed amps -- badly so
         * for sub-amp motors (e.g. board 0.43 A vs a 2 A default -> ~4.6x). Read
         * it straight from the slave on bringup so the two can never disagree. */
        uint32_t rcur_ma = 0U;
        QString  rerr;
        if (m_can.readSdo(idx, 0x6075, 0, &rcur_ma, 4, &rerr) == 0 && rcur_ma > 0U){
            m_ratedA = static_cast<float>(rcur_ma) / 1000.0f;
            emit info(QStringLiteral("slave %1 telemetry current scale <- 0x6075 = %2 A")
                      .arg(idx).arg(double(m_ratedA), 0, 'g', 4));
        }
    }
}

void MasterWorker::enableOne(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    m_can.setCycleControlword(static_cast<uint32_t>(idx), 0);
    if (master_mgr_enable_one(m_mgr, idx) != 0){
        emit error(QStringLiteral("enable slave %1 failed").arg(idx));
    } else {
        m_can.setCycleControlword(static_cast<uint32_t>(idx), kCwEnableOperation);
        emit controlwordCached(idx, kCwEnableOperation);
    }
}

void MasterWorker::disableOne(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Cache cw=0x0000 (Disable Voltage) so the cycle's RPDOs stop
     * holding OPERATION_ENABLED, then run the SDK's SDO disable walk.
     * Mutex blocks onCycleTick during the walk, so no race. */
    m_can.setCycleControlword(static_cast<uint32_t>(idx), 0);
    emit controlwordCached(idx, 0);
    if (master_mgr_disable_one(m_mgr, idx) != 0){
        emit error(QStringLiteral("disable slave %1 failed").arg(idx));
    }
}

void MasterWorker::customSdoRead(int slaveIdx, uint16_t odIdx, uint8_t sub, int byteLen)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit customSdoDone(slaveIdx, /*isWrite=*/false, odIdx, sub, false,
                           QString(), QStringLiteral("not connected"));
        return;
    }
    if (byteLen <= 0 || byteLen > 8){
        emit customSdoDone(slaveIdx, false, odIdx, sub, false,
                           QString(),
                           QStringLiteral("byteLen must be 1..8 for expedited SDO"));
        return;
    }
    uint8_t buf[8] = {0};
    QString err;
    const int rc = m_can.readSdo(slaveIdx, odIdx, sub, buf, byteLen, &err);
    if (rc != 0){
        emit customSdoDone(slaveIdx, false, odIdx, sub, false, QString(), err);
        return;
    }
    /* Format as little-endian unsigned hex so the operator sees the
     * canonical wire value (matches how SDO bytes get laid out). */
    QString hex = QStringLiteral("0x");
    for (int i = byteLen - 1; i >= 0; --i){
        hex += QStringLiteral("%1").arg(buf[i], 2, 16, QChar('0'));
    }
    emit customSdoDone(slaveIdx, false, odIdx, sub, true, hex, QString());
}

void MasterWorker::customSdoWrite(int slaveIdx, uint16_t odIdx, uint8_t sub,
                                  QByteArray bytes)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit customSdoDone(slaveIdx, /*isWrite=*/true, odIdx, sub, false,
                           QString(), QStringLiteral("not connected"));
        return;
    }
    if (bytes.isEmpty() || bytes.size() > 8){
        emit customSdoDone(slaveIdx, true, odIdx, sub, false, QString(),
                           QStringLiteral("payload size must be 1..8 bytes"));
        return;
    }
    QString err;
    const int rc = m_can.writeSdo(slaveIdx, odIdx, sub,
                                  bytes.constData(),
                                  static_cast<uint32_t>(bytes.size()), &err);
    emit customSdoDone(slaveIdx, true, odIdx, sub,
                       /*ok=*/(rc == 0), QString(),
                       rc == 0 ? QString() : err);
}

void MasterWorker::walkControlwordOne(int idx, uint16_t cw)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Latch the requested cw into the cycle cache so every cycle RPDO
     * carries it. Operator is intentionally hand-driving the FSM. */
    m_can.setCycleControlword(static_cast<uint32_t>(idx), cw);
    emit controlwordCached(idx, cw);
    emit info(QStringLiteral("walk slave %1: cw=0x%2")
              .arg(idx).arg(cw, 4, 16, QChar('0')));
}

void MasterWorker::quickStopOne(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Latch cw=0x0002 (Enable Voltage + Quick Stop bit cleared) into
     * the cycle cache. Slave begins controlled decel using its
     * Quick-stop deceleration (0x6085) and ends in QUICK_STOP_ACTIVE.
     * The cycle keeps holding 0x0002 so the slave doesn't bounce
     * back to OPERATION_ENABLED on the next RPDO. */
    m_can.setCycleControlword(static_cast<uint32_t>(idx), kCwQuickStop);
    emit controlwordCached(idx, kCwQuickStop);
    emit info(QStringLiteral("quick stop slave %1 (cw=0x%2)")
              .arg(idx).arg(kCwQuickStop, 4, 16, QChar('0')));
}

void MasterWorker::setBrake(int idx, bool engaged)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    m_can.setBrake(static_cast<uint32_t>(idx), engaged);
    emit info(QStringLiteral("slave %1 brake %2")
                  .arg(idx).arg(engaged ? "ENGAGED" : "released"));
}

void MasterWorker::faultReset(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    /* Fault reset transitions Fault → SwitchOnDisabled; the operator
     * must re-bringup before commands flow again. Park the cycle cw
     * at 0x0000 while the SDK pulses the FaultReset bit via SDO. */
    m_can.setCycleControlword(static_cast<uint32_t>(idx), 0);
    emit controlwordCached(idx, 0);
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
    } else {
        const char* name = (mode == Mode::Position) ? "POSITION" :
                           (mode == Mode::Velocity) ? "VELOCITY" :
                           (mode == Mode::Torque)   ? "TORQUE"   : "NONE";
        emit info(QStringLiteral("slave %1 mode → %2 (0x6060 = %3)")
                      .arg(idx).arg(name).arg(int(toIntfMode(mode))));
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
    /* Zero the inactive target fields so a stale setpoint from a
     * previous mode (e.g. velocity 100 rad/s left over after switching
     * to position) doesn't bleed into the RPDO. apply_cia402_targets
     * on the sim writes ALL THREE vmotor targets every cycle from the
     * RPDO payload — only the active mode's target is consumed by
     * vmotor_step, but mode races at switch time can pick up the
     * wrong field and cause the motor to run aimlessly. */
    auto& c = m_cmdCache[idx];
    c.last = which;
    switch (which){
    case TargetKind::Position: c.pos = value; c.vel = 0.0f; c.trq = 0.0f; break;
    case TargetKind::Velocity: c.vel = value; c.pos = 0.0f; c.trq = 0.0f; break;
    case TargetKind::Torque:   c.trq = value; c.pos = 0.0f; c.vel = 0.0f; break;
    }

    /* Push the latest cached targets into the per-tick PDO cache so
     * the next cycle frame carries them. Reverse of the per-tick SI
     * conversion in onTick — via cia402_utils helpers. */
    const int32_t pos_counts = cia402_rad_to_inc(c.pos, m_countsPerRev);
    const int32_t vel_counts = cia402_rad_s_to_inc_s(c.vel, m_countsPerRev);
    const int16_t trq_perm   = cia402_nm_to_milli_rated(c.trq, m_ratedNm);
    m_can.setTarget(static_cast<uint32_t>(idx), pos_counts, vel_counts, trq_perm);
    /* No info-log here — live-stream slider can call this at 50 Hz,
     * which spams the LogDock and drives sustained UI-thread lag. The
     * RPDO log was a one-shot debugging aid that's served its purpose. */
}

void MasterWorker::setScaling(double countsPerRev, double ratedNm, double ratedA)
{
    QMutexLocker lock(&m_mutex);
    if (countsPerRev > 0.0){ m_countsPerRev = static_cast<uint32_t>(countsPerRev + 0.5); }
    if (ratedNm      > 0.0){ m_ratedNm      = static_cast<float>(ratedNm); }
    if (ratedA       > 0.0){ m_ratedA       = static_cast<float>(ratedA); }
    emit info(QStringLiteral("scaling: %1 inc/rev, rated %2 Nm / %3 A")
                  .arg(m_countsPerRev).arg(m_ratedNm, 0, 'g', 4)
                  .arg(m_ratedA, 0, 'g', 4));
}

/* --- Open-loop V/f (unified vendor 0x2030 -- type + freq + amp) ------- *
 *
 * Layout (board side, app_mfr_od.h):
 *   :01 type  UINT8  (0 = disable, 1 = BSP raw-gen, 2 = FOC mode -2)
 *   :02 freq  INT32  (electrical mHz)
 *   :03 amp   UINT32 (milli-pu of Vbus)
 *
 * For FOC type=2 the board doesn't engage the cascade until 0x6060 == -2
 * is also written; the type byte just selects which engine consumes the
 * freq/amp pair. For BSP type=1 the write hook fires bsp_vf_arm + set
 * directly. Disable (type=0) disarms BSP and idles the FOC path. */

void MasterWorker::setVfSetpoint(int idx, int engine, double freqHz, double level)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    const double lv = (level < 0.0) ? 0.0 : level;
    int32_t  fmhz   = static_cast<int32_t>(std::lround(freqHz * 1000.0));
    uint32_t amilli = static_cast<uint32_t>(std::lround(lv * 1000.0));
    QString  err;
    int rc = (m_can.writeSdo(idx, 0x2030, 2, &fmhz,   4, &err) != 0 ||
              m_can.writeSdo(idx, 0x2030, 3, &amilli, 4, &err) != 0) ? -1 : 0;
    if (rc != 0){
        emit error(QStringLiteral("V/f setpoint slave %1 — %2").arg(idx).arg(err));
    }
    (void)engine;   /* freq/amp shared; engine still chosen by type at start time */
}

void MasterWorker::startVfOpenLoop(int idx, int engine, double freqHz, double level)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ emit error(QStringLiteral("start V/f: not connected")); return; }
    const double lv = (level < 0.0) ? 0.0 : level;
    int32_t  fmhz   = static_cast<int32_t>(std::lround(freqHz * 1000.0));
    uint32_t amilli = static_cast<uint32_t>(std::lround(lv * 1000.0));
    QString  err;
    if (engine == 1){
        /* BSP raw generator: stage freq/amp, then set type=1 to arm. */
        uint8_t type = 1u;
        if (m_can.writeSdo(idx, 0x2030, 2, &fmhz,   4, &err) != 0 ||
            m_can.writeSdo(idx, 0x2030, 3, &amilli, 4, &err) != 0 ||
            m_can.writeSdo(idx, 0x2030, 1, &type,   1, &err) != 0){
            emit error(QStringLiteral("start BSP V/f: slave %1 — %2").arg(idx).arg(err));
            return;
        }
        emit info(QStringLiteral("BSP V/f on slave %1: %2 Hz, %3 pu")
                      .arg(idx).arg(freqHz, 0, 'g', 4).arg(lv, 0, 'g', 4));
    } else {
        /* FOC path: type=2 + freq/amp, then 0x6060=-2 to activate cascade. */
        uint8_t type = 2u;
        int16_t mode = -2;
        if (m_can.writeSdo(idx, 0x2030, 1, &type,   1, &err) != 0 ||
            m_can.writeSdo(idx, 0x2030, 2, &fmhz,   4, &err) != 0 ||
            m_can.writeSdo(idx, 0x2030, 3, &amilli, 4, &err) != 0 ||
            m_can.writeSdo(idx, 0x6060, 0, &mode,   2, &err) != 0){
            emit error(QStringLiteral("start V/f: slave %1 — %2").arg(idx).arg(err));
            return;
        }
        /* Diagnostic read-back so FOC-doesn't-spin is debuggable from the
         * log: 0x6061 must be -2, 0x6041 low byte should show
         * OPERATION_ENABLED (...0111 / e.g. 0x0237). */
        int16_t  md = 0;  uint16_t sw = 0;  QString rerr;
        (void)m_can.readSdo(idx, 0x6061, 0, &md, 2, &rerr);
        (void)m_can.readSdo(idx, 0x6041, 0, &sw, 2, &rerr);
        emit info(QStringLiteral("FOC V/f on slave %1: %2 Hz, %3 pu  "
                                 "[0x6061 mode=%4 (want -2), statusword=0x%5]")
                      .arg(idx).arg(freqHz, 0, 'g', 4).arg(lv, 0, 'g', 4)
                      .arg(md).arg(sw, 4, 16, QLatin1Char('0')));
    }
}

void MasterWorker::stopVfOpenLoop(int idx, int engine)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){ return; }
    QString err;
    /* Unified stop: type=0 disarms BSP + idles FOC. For FOC also clear
     * 0x6060 so the next operator action sees mode NONE. */
    uint8_t type = 0u;
    if (m_can.writeSdo(idx, 0x2030, 1, &type, 1, &err) != 0){
        emit error(QStringLiteral("stop V/f: slave %1 — %2").arg(idx).arg(err));
        return;
    }
    if (engine != 1){
        int16_t mode = 0;
        if (m_can.writeSdo(idx, 0x6060, 0, &mode, 2, &err) != 0){
            emit error(QStringLiteral("stop V/f: clear mode slave %1 — %2").arg(idx).arg(err));
            return;
        }
    }
    emit info(QStringLiteral("V/f stopped on slave %1 (%2)")
                  .arg(idx).arg(engine == 1 ? "BSP" : "FOC"));
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

    /* Park every slot's cycle controlword at 0x0000 (Disable Voltage)
     * BEFORE the SDO disable walk. Without this, the cycle would keep
     * streaming 0x000F between the SDK's writes and the slave would
     * snap back to OPERATION_ENABLED. Mutex blocks the cycle during
     * the walk; the cw=0 latch makes sure post-walk RPDOs are safe too. */
    const uint32_t n_slots = m_can.slaveCount();
    for (uint32_t i = 0; i < n_slots; ++i){
        m_can.setCycleControlword(i, 0);
        emit controlwordCached(static_cast<int>(i), 0);
    }
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

void MasterWorker::tuneGain(int idx, Loop loop, float bw_hz)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit gainTuned(idx, loop, 0.0f, 0.0f, false);
        return;
    }
    auto* slave = master_mgr_get_slave(m_mgr, idx);
    if (!slave){
        emit gainTuned(idx, loop, 0.0f, 0.0f, false);
        return;
    }
    float32_t kp = 0.0f, ki = 0.0f;
    /* Conservative timeouts. The board's tune service is non-blocking on
     * its own (a single motor_tune_*_pi call returns in microseconds), but
     * the master polls 0x2080:04 status -- 2 s of polling at 25 ms/cycle
     * = 80 cycles, plenty of margin. */
    const int rc = motor_drive_intf_tune_bw(slave, toIntfLoop(loop),
                                              bw_hz, &kp, &ki,
                                              /*timeout_ms=*/2000u,
                                              /*poll_ms=*/25u);
    const bool ok = (rc == 0);
    if (!ok){
        emit error(QStringLiteral("tune gain slave %1 loop %2 bw %3 Hz failed (rc=%4)")
                       .arg(idx).arg(int(loop)).arg(bw_hz, 0, 'f', 1).arg(rc));
    } else {
        emit info(QStringLiteral("slave %1 loop %2 tuned at %3 Hz -> kp:%4 ki:%5")
                      .arg(idx).arg(int(loop)).arg(bw_hz, 0, 'f', 1)
                      .arg(kp, 0, 'g', 4).arg(ki, 0, 'g', 4));
    }
    emit gainTuned(idx, loop, kp, ki, ok);
}

void MasterWorker::captureStep(int idx, Loop loop, float amp, float ref_default)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit stepCaptured(idx, loop, {}, {}, 0.0f, false);
        return;
    }
    auto* slave = master_mgr_get_slave(m_mgr, idx);
    if (!slave){
        emit stepCaptured(idx, loop, {}, {}, 0.0f, false);
        return;
    }
    /* 256 samples per buffer. The board fills them on its ISR (TORQUE
     * mode at 20 kHz = 12.8 ms window, SPEED mode at 2 kHz = 128 ms);
     * 5 s timeout is comfortable even with master-side polling latency. */
    QVector<float> buf0(256), buf1(256);
    uint32_t sample_rate_hz = 0u;
    const int rc = motor_drive_intf_capture_step(
        slave, toIntfLoop(loop), amp, ref_default,
        buf0.data(), buf1.data(), &sample_rate_hz,
        /*timeout_ms=*/5000u, /*poll_ms=*/50u);
    const bool ok = (rc == 0);
    if (!ok){
        emit error(QStringLiteral("step capture slave %1 loop %2 amp %3 failed (rc=%4)")
                       .arg(idx).arg(int(loop)).arg(amp, 0, 'f', 3).arg(rc));
        emit stepCaptured(idx, loop, {}, {}, 0.0f, false);
        return;
    }
    emit info(QStringLiteral("slave %1 loop %2 step amp %3 captured @ %4 Hz")
                  .arg(idx).arg(int(loop)).arg(amp, 0, 'f', 3).arg(sample_rate_hz));
    emit stepCaptured(idx, loop, buf0, buf1,
                       float(sample_rate_hz), true);
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
    bool     readonly;     /**< write pass skips RO entries (e.g. 0x6078) */
};

template <typename T>
static DriveField fld(uint16_t i, uint8_t s, T& v)
{
    return { i, s, uint8_t(sizeof(T)), &v, /*readonly=*/false };
}
template <typename T>
static DriveField fldRO(uint16_t i, uint8_t s, T& v)
{
    return { i, s, uint8_t(sizeof(T)), &v, /*readonly=*/true };
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
        /* 0x6076 (rated torque) is read-only on the drive: derived =
         * Kt * rated_current. Read it for display/scaling, skip on write. */
        fldRO(0x6076, 0, c.rated_torque),    /* mNm, RO                    */
        fld(0x6075, 0, c.rated_current),     /* mA                         */
        fld(0x608F, 1, c.enc_increments),    /* encoder increments         */
        fld(0x608F, 2, c.enc_motor_revs),    /* motor revolutions          */
        fld(0x6091, 1, c.gear_ratio_motor_revs), /* gear motor revs        */
        fld(0x6091, 2, c.gear_ratio_shaft_revs), /* gear shaft revs        */

        /* Protection (vendor 0x2050). Current actual (0x6078) moved to the
         * telemetry graph (streamed in the TPDO). */
        fld(0x2050, 1, c.stall_current),
        fld(0x2050, 2, c.stall_time),

        /* Manufacturer range (0x20xx): node-id + current-sensor cal
         * (offset 0x2040 + gain 0x2041, RW) + Hall offset (0x2060) +
         * motor-profile electrical record (0x2070). */
        fld(0x2000, 1, c.node_id),
        fld(0x2040, 1, c.current_offset_a),
        fld(0x2040, 2, c.current_offset_b),
        fld(0x2040, 3, c.current_offset_c),
        fld(0x2041, 1, c.current_gain_a),
        fld(0x2041, 2, c.current_gain_b),
        fld(0x2041, 3, c.current_gain_c),
        fld(0x2060, 0, c.hall_offset),
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
    int written = 0;
    for (const auto& f : fields){
        if (f.readonly){ continue; }   /* skip current_actual etc. */
        QString err;
        const int rc = m_can.writeSdo(idx, f.idx, f.sub, f.ptr, f.len, &err);
        if (rc != 0){ failures << err; }
        else         { ++written; }
    }
    if (failures.isEmpty()){
        emit info(QStringLiteral("drive config: slave %1 written (%2 fields)")
                      .arg(idx).arg(written));
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

void MasterWorker::writeMotorProfile(int idx, MotorParams mp)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit error(QStringLiteral("write motor profile: not connected"));
        return;
    }
    QStringList fails;
    auto wr = [&](uint16_t i, uint8_t s, const void* p, uint32_t n,
                  const QString& nm){
        QString err;
        if (m_can.writeSdo(idx, i, s, p, n, &err) != 0){ fails << nm; }
    };

    const uint32_t type = static_cast<uint32_t>(mp.type);
    const uint32_t pole = mp.pole_pair;
    const int32_t  rvol = mp.rated_vol;
    const uint32_t rcur = static_cast<uint32_t>(std::lround(mp.rated_cur * 1000.0f));   /* mA  */
    const int32_t  rspd = mp.rated_speed;                                               /* rpm */

    /* 0x6076 (rated torque) is NOT written: the drive derives it as
     * Kt * rated_current (Kt from flux/pole at 0x2070:2/6, rated current at
     * 0x6075), so 0x6076 is read-only. Writing flux/pole + rated current is
     * sufficient for the drive to recompute it. */
    wr(0x2070, 1, &type,          4, QStringLiteral("0x2070:1"));
    wr(0x2070, 2, &pole,          4, QStringLiteral("0x2070:2"));
    wr(0x2070, 3, &mp.rs,         4, QStringLiteral("0x2070:3"));
    wr(0x2070, 4, &mp.ls_d,       4, QStringLiteral("0x2070:4"));
    wr(0x2070, 5, &mp.ls_q,       4, QStringLiteral("0x2070:5"));
    wr(0x2070, 6, &mp.rated_flux, 4, QStringLiteral("0x2070:6"));
    wr(0x2070, 7, &mp.inertia,    4, QStringLiteral("0x2070:7"));
    wr(0x2070, 8, &rvol,          4, QStringLiteral("0x2070:8"));
    /* 0x2070:9 torque constant Kt (Nm/A). 0 -> the drive derives it from
     * pole/flux; non-zero overrides it (needed for BLDC). */
    wr(0x2070, 9,  &mp.torque_constant, 4, QStringLiteral("0x2070:9"));
    wr(0x2070, 10, &rspd,               4, QStringLiteral("0x2070:10"));  /* rated speed */
    wr(0x6075, 0,  &rcur,               4, QStringLiteral("0x6075"));

    /* 0x2070:11 encoder CPR (= 4*lines) exists only on the incremental-
     * encoder variant (3FL_2); best-effort so it never fails the profile on
     * Hall / abs-encoder nodes that don't expose the sub. */
    {
        const uint32_t cpr = mp.cpr;
        QString e;
        if (m_can.writeSdo(idx, 0x2070, 11, &cpr, 4, &e) != 0){
            emit info(QStringLiteral("slave %1: 0x2070:11 (CPR) skipped — %2")
                          .arg(idx).arg(e));
        }
    }

    if (fails.isEmpty()){
        emit info(QStringLiteral("motor profile written to slave %1").arg(idx));
    } else {
        emit error(QStringLiteral("motor profile: slave %1 — failed: %2")
                       .arg(idx).arg(fails.join(QStringLiteral(", "))));
    }
}

void MasterWorker::readMotorProfile(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit motorProfileRead(idx, MotorParams{}, false,
                              QStringLiteral("not connected"));
        return;
    }
    MotorParams mp{};
    QStringList fails;
    auto rd = [&](uint16_t i, uint8_t s, void* p, uint32_t n, const QString& nm){
        QString err;
        if (m_can.readSdo(idx, i, s, p, n, &err) != 0){ fails << nm; return false; }
        return true;
    };

    uint32_t type = 0, pole = 0;
    int32_t  rvol = 0;
    rd(0x2070, 1, &type,            4, QStringLiteral("0x2070:1"));
    rd(0x2070, 2, &pole,            4, QStringLiteral("0x2070:2"));
    rd(0x2070, 3, &mp.rs,           4, QStringLiteral("0x2070:3"));
    rd(0x2070, 4, &mp.ls_d,         4, QStringLiteral("0x2070:4"));
    rd(0x2070, 5, &mp.ls_q,         4, QStringLiteral("0x2070:5"));
    rd(0x2070, 6, &mp.rated_flux,   4, QStringLiteral("0x2070:6"));
    rd(0x2070, 7, &mp.inertia,      4, QStringLiteral("0x2070:7"));
    rd(0x2070, 8, &rvol,            4, QStringLiteral("0x2070:8"));
    rd(0x2070, 9,  &mp.torque_constant, 4, QStringLiteral("0x2070:9"));
    int32_t rspd = 0;
    rd(0x2070, 10, &rspd,            4, QStringLiteral("0x2070:10"));   /* rated speed */
    mp.type        = (type == static_cast<uint32_t>(MotorType::Pmsm))
                         ? MotorType::Pmsm : MotorType::Bldc;
    mp.pole_pair   = pole;
    mp.rated_vol   = rvol;
    mp.rated_speed = rspd;

    /* 0x2070:11 CPR only exists on the encoder variant (3FL_2); best-effort. */
    { uint32_t cpr = 0; QString e;
      if (m_can.readSdo(idx, 0x2070, 11, &cpr, 4, &e) == 0){ mp.cpr = cpr; } }

    /* Rated current (0x6075, mA -> float A) + derived rated torque
     * (0x6076, mNm -> Nm, RO). */
    uint32_t rcur_ma = 0, rtrq_mnm = 0;
    if (rd(0x6075, 0, &rcur_ma,  4, QStringLiteral("0x6075"))){
        mp.rated_cur = static_cast<float>(rcur_ma) / 1000.0f;
    }
    if (rd(0x6076, 0, &rtrq_mnm, 4, QStringLiteral("0x6076"))){
        mp.rated_torque = static_cast<float>(rtrq_mnm) / 1000.0f;
    }

    const bool ok = fails.isEmpty();
    emit motorProfileRead(idx, mp, ok,
        ok ? QStringLiteral("motor profile read from slave %1").arg(idx)
           : QStringLiteral("motor profile read: slave %1 — failed: %2")
                 .arg(idx).arg(fails.join(QStringLiteral(", "))));
}

void MasterWorker::readDeviceInfo(int idx)
{
    QMutexLocker lock(&m_mutex);
    if (!m_mgr){
        emit deviceInfoRead(idx, DeviceInfo{}, false,
                            QStringLiteral("not connected"));
        return;
    }
    DeviceInfo di{};
    int ok = 0;
    QStringList missing;

    auto rdU32 = [&](uint16_t i, uint8_t s, uint32_t& out, const char* nm){
        QString err;
        if (m_can.readSdo(idx, i, s, &out, 4, &err) == 0){ ++ok; }
        else { missing << QString::fromLatin1(nm); }
    };
    auto rdStr = [&](uint16_t i, QString& out, const char* nm){
        /* Device strings are short. Keep the requested length small: a
         * large request (e.g. 63 B) overflows the USDO expedited single
         * frame and the upload aborts even though the object is tiny. */
        char buf[32] = { 0 };
        QString err;
        if (m_can.readSdo(idx, i, 0, buf, sizeof(buf) - 1, &err) == 0){
            buf[sizeof(buf) - 1] = '\0';
            out = QString::fromLatin1(buf).trimmed();
            ++ok;
        } else { missing << QString::fromLatin1(nm); }
    };

    rdU32(0x1000, 0, di.device_type,  "0x1000 device type");
    rdStr(0x1008,    di.device_name,  "0x1008 device name");
    rdStr(0x1009,    di.hw_version,   "0x1009 hw version");
    rdStr(0x100A,    di.sw_version,   "0x100A sw version");
    rdU32(0x1018, 1, di.vendor_id,    "0x1018:1 vendor id");
    rdU32(0x1018, 2, di.product_code, "0x1018:2 product code");
    rdU32(0x1018, 3, di.revision,     "0x1018:3 revision");
    rdU32(0x1018, 4, di.serial,       "0x1018:4 serial");

    const bool anyOk = (ok > 0);
    const QString msg = missing.isEmpty()
        ? QString()
        : QStringLiteral("Not read: %1").arg(missing.join(QStringLiteral(", ")));
    if (anyOk){
        emit info(QStringLiteral("device info: slave %1 - %2/8 objects read")
                      .arg(idx).arg(ok));
    } else {
        emit error(QStringLiteral("device info: slave %1 - no identity objects "
                                  "readable").arg(idx));
    }
    emit deviceInfoRead(idx, di, anyOk, msg);
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
    int16_t  mode = 6;   /* 0x6060 is declared 2-byte in the cia402 OD */
    uint16_t cw0  = 0x000F;
    uint16_t cw1  = 0x001F;
    QString  err;
    if (m_can.writeSdo(idx, 0x6060, 0, &mode, 2, &err) != 0 ||
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
    /* Same defensive zero as setTarget — keep inactive fields at 0 so
     * a leftover velocity/torque from a previous run doesn't get sent
     * alongside the active generator channel. */
    switch (m_genCfg.target){
    case TargetKind::Position: c.pos = value; c.vel = 0.0f; c.trq = 0.0f; break;
    case TargetKind::Velocity: c.vel = value; c.pos = 0.0f; c.trq = 0.0f; break;
    case TargetKind::Torque:   c.trq = value; c.pos = 0.0f; c.vel = 0.0f; break;
    }

    /* Push the latest cached targets into the per-tick PDO TX cache
     * so the keep-alive frame carries them on the wire. Without this
     * the generator only touched the OD via SDO, which the sim does
     * NOT poll — its vmotor only adopts new targets when an RPDO
     * arrives (apply_cia402_targets in vrmc_sim.c), so the
     * step / sine / chirp signals never reached the plant and the
     * Tuning charts showed flat lines. Same dance as
     * MasterWorker::setTarget above. */
    const int32_t pos_counts = cia402_rad_to_inc(c.pos, m_countsPerRev);
    const int32_t vel_counts = cia402_rad_s_to_inc_s(c.vel, m_countsPerRev);
    const int16_t trq_perm   = cia402_nm_to_milli_rated(c.trq, m_ratedNm);
    m_can.setTarget(static_cast<uint32_t>(m_genIdx),
                    pos_counts, vel_counts, trq_perm);
}

}  // namespace vrmc
