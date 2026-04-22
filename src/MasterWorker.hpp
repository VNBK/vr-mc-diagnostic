/**
 * @file   MasterWorker.hpp
 * @brief  QObject that owns the C master stack and runs its I/O loop.
 *
 * Lives on its own QThread. All calls into the vr-mc-sdk C APIs
 * (@c master_mgr_*, @c motor_drive_intf_*) happen here. UI talks to it
 * through signals/slots — never via direct pointer access.
 */

#pragma once

#include "backends/CanBackend.hpp"

#include <QHash>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QVector>
#include <memory>

QT_FORWARD_DECLARE_CLASS(QTimer)

extern "C" {
#include "master_mgr.h"
}

namespace vrmc {

/** One row of telemetry emitted after each refresh. */
struct SlaveSnapshot
{
    int      idx        = -1;
    int      id         = 0;
    int      state      = 0;    /**< motor_intf_state_t                 */
    bool     online     = false;
    float    position   = 0.0f;
    float    velocity   = 0.0f;
    float    torque     = 0.0f;
    /* PDO-only fields (raw 16-bit CiA 402 statusword + error register).
     * Populated when the backend has seen at least one TPDO for this
     * slave. */
    bool     pdoFresh   = false;
    uint16_t statusword = 0;
    uint16_t errorCode  = 0;
    uint64_t pdoRxCount = 0;
    /* Extended channels — read from the drive when the backend exposes
     * them (motor_drive_intf_get_current / _temperature); otherwise
     * derived from torque / velocity so the diagnostic has something to
     * plot against a minimal simulator. */
    float    current     = 0.0f; /**< A  */
    float    temperature = 0.0f; /**< °C */
    /* Last commanded setpoints (what the worker last pushed, or 0). */
    float    cmdPosition = 0.0f;
    float    cmdVelocity = 0.0f;
    float    cmdTorque   = 0.0f;
    /* Tracking error for the last-active command channel. */
    float    trackingError = 0.0f;
    QString  name;
};

/** Command-target selector mirroring @c master_mgr_target_t. */
enum class TargetKind { Position = 0, Velocity = 1, Torque = 2 };

/** Mode selector mirroring @c motor_intf_mode_t. */
enum class Mode {
    None     = 0,
    Torque   = 1,
    Velocity = 2,
    Position = 3,
};

/** Control-loop selector mirroring @c motor_intf_loop_t. */
enum class Loop {
    Current  = 0,
    Velocity = 1,
    Position = 2,
};

/** Runtime-configurable drive parameters backed by the CiA 402 OD.
 *  The indices below are canonical (see CiA 402 §7) — real drives expose
 *  them; the in-tree simulator only answers a subset so reads/writes
 *  against it often abort. */
struct DriveConfig
{
    /* Homing (0x6098 / 0x6099 / 0x607C / 0x609A). */
    int8_t   homing_method      = 0;
    int32_t  home_offset        = 0;
    uint32_t homing_speed_fast  = 100;   /**< switch-search speed  */
    uint32_t homing_speed_slow  = 10;    /**< zero-search speed    */
    uint32_t homing_accel       = 1000;

    /* Motion profile (0x6081 / 0x6083 / 0x6084 / 0x6085). */
    uint32_t profile_velocity   = 1000;
    uint32_t profile_accel      = 5000;
    uint32_t profile_decel      = 5000;
    uint32_t quickstop_decel    = 10000;

    /* Protection (0x6065 / 0x6066 / 0x607D / 0x6080 / 0x6072). */
    uint32_t following_error    = 5000;
    uint16_t following_error_ms = 100;
    int32_t  pos_limit_min      = -2147483647;
    int32_t  pos_limit_max      =  2147483647;
    uint32_t max_motor_speed    = 10000;
    uint16_t max_torque         = 2000;  /**< per-mille of rated  */

    /* Encoder / scaling (0x608F:1 / 0x6091 / 0x6092). */
    uint32_t enc_resolution     = 16384; /**< counts per rev       */
    uint32_t gear_num           = 1;
    uint32_t gear_den           = 1;
    uint32_t feed_const_num     = 1;
    uint32_t feed_const_den     = 1;
};

/** Signal-generator waveform. */
struct GenCfg
{
    enum Shape { Constant = 0, Step, Ramp, Sine, Chirp };
    Shape      shape       = Step;
    TargetKind target      = TargetKind::Position;
    float      amplitude   = 1.0f;    /**< peak                              */
    float      offset      = 0.0f;    /**< DC bias                           */
    float      frequency   = 1.0f;    /**< Hz (sine); start freq (chirp)     */
    float      frequencyEnd = 5.0f;   /**< Hz end for chirp                  */
    float      durationSec = 5.0f;    /**< 0 = run until Stop                */
    int        rateHz      = 50;      /**< setpoint update rate              */
};

class MasterWorker : public QObject
{
    Q_OBJECT
public:
    explicit MasterWorker(QObject* parent = nullptr);
    ~MasterWorker() override;

public slots:
    void connectCan (const vrmc::CanConfig& cfg);
    void disconnect_();

    void refreshOnce();            /**< explicit poke; normally auto-periodic */
    void setRefreshHz(int hz);     /**< default 20 Hz */

    void bringupOne (int idx, uint32_t timeoutMs);
    void enableOne  (int idx);
    void disableOne (int idx);
    void faultReset (int idx);
    void setMode    (int idx, vrmc::Mode mode);
    void setTarget  (int idx, vrmc::TargetKind which, float value);
    void setId      (int idx, uint8_t newId);

    /* Emergency stop: disable every registered slave at once. */
    void disableAll ();

    /* Gain editor I/O. Response lands on @c gainRead signal. */
    void readGain   (int idx, vrmc::Loop loop);
    void writeGain  (int idx, vrmc::Loop loop, float kp, float ki);

    /* Signal generator driven off a worker-thread QTimer. */
    void startGenerator(int idx, vrmc::GenCfg cfg);
    void stopGenerator (int idx);

    /* Push a new PDO mapping (TPDO1 or RPDO1) to a slave via SDO. */
    void applyPdoMapping(int idx, bool isTpdo,
                         QVector<vrmc::CanBackend::PdoMapEntry> entries);

    /* Drive-parameter batch read/write (homing, motion profile, limits,
     * scaling). Both serialise SDO ops against the worker's mutex, so
     * they'll queue behind the periodic refresh. */
    void readDriveConfig (int idx);
    void writeDriveConfig(int idx, vrmc::DriveConfig cfg);

    /** Kick off a CiA 402 homing sequence on a slave.
     *  Switches the drive into homing mode and pulses controlword bit 4
     *  (rising edge = start homing). The drive is expected to already be
     *  in Operation Enabled — caller typically hits Enable first. */
    void startHoming     (int idx);

    /** One-click encoder zero: reads current position_actual (0x6064)
     *  and writes it into home_offset (0x607C) so subsequent commanded
     *  positions reference "here" as the origin. */
    void zeroEncoderHere (int idx);

    /** Companion to @ref zeroEncoderHere: reads current torque_actual
     *  (0x6077) and writes it into torque_offset (0x60B2), effectively
     *  taring the torque feedforward channel at the current loading. */
    void zeroTorqueHere  (int idx);

signals:
    void connected(int slaveCount);
    void disconnected();
    void error(const QString& msg);
    void info (const QString& msg);
    void snapshots(QVector<vrmc::SlaveSnapshot> snap);
    void gainRead (int idx, vrmc::Loop loop, float kp, float ki, bool ok);
    void generatorStarted(int idx);
    void generatorStopped(int idx);
    void pdoMappingApplied(int idx, bool isTpdo, bool ok, QString message);
    void driveConfigRead   (int idx, vrmc::DriveConfig cfg, bool ok,
                            QString message);
    void driveConfigWritten(int idx, bool ok, QString message);
    /** Encoder-zero / torque-tare completion with the raw value that was
     *  captured (and subsequently written). Dialog uses this to confirm
     *  the action rather than waiting on the next SDO refresh. */
    void calibrationDone   (int idx, QString what, int64_t raw, bool ok,
                            QString message);

private slots:
    void onTick();                  /**< QTimer callback running refresh + pump */
    void onGenTick();               /**< QTimer callback pushing waveform value */

private:
    void teardown();

    CanBackend                  m_can;
    master_mgr_t*               m_mgr   = nullptr;
    QTimer*                     m_tick  = nullptr;
    int                         m_hz    = 100;   /* PDO-driven; safe to raise */
    int                         m_refreshTick = 0; /* SDO housekeeping divider */
    mutable QMutex              m_mutex;

    /* Active generator state. Only one generator at a time for V1. */
    QTimer*                     m_genTimer = nullptr;
    GenCfg                      m_genCfg;
    int                         m_genIdx   = -1;
    double                      m_genT0    = 0.0;

    /* Last commanded setpoint per slave (indexed by slave idx). */
    struct CmdCache { float pos = 0.0f; float vel = 0.0f; float trq = 0.0f;
                      TargetKind last = TargetKind::Position; };
    QHash<int, CmdCache>        m_cmdCache;
};

}  // namespace vrmc

Q_DECLARE_METATYPE(vrmc::SlaveSnapshot)
Q_DECLARE_METATYPE(QVector<vrmc::SlaveSnapshot>)
Q_DECLARE_METATYPE(vrmc::CanConfig)
Q_DECLARE_METATYPE(vrmc::Mode)
Q_DECLARE_METATYPE(vrmc::TargetKind)
Q_DECLARE_METATYPE(vrmc::Loop)
Q_DECLARE_METATYPE(vrmc::GenCfg)
Q_DECLARE_METATYPE(vrmc::CanBackend::PdoMapEntry)
Q_DECLARE_METATYPE(QVector<vrmc::CanBackend::PdoMapEntry>)
Q_DECLARE_METATYPE(vrmc::DriveConfig)
