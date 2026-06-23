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
#include "MotorProfile.hpp"      /* MotorParams (writeMotorProfile) */

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

    /* Protection (0x6065 / 0x6066 / 0x607D / 0x6080 / 0x6072 / 0x6076). */
    uint32_t following_error    = 5000;
    uint16_t following_error_ms = 100;
    int32_t  pos_limit_min      = -2147483647;
    int32_t  pos_limit_max      =  2147483647;
    uint32_t max_motor_speed    = 10000;
    uint16_t max_torque         = 2000;  /**< per-mille of rated  */
    uint32_t rated_torque       = 0;     /**< 0x6076, mNm                  */
    uint32_t rated_current      = 0;     /**< 0x6075, mA                   */

    /* Position encoder resolution (0x608F record, like a gearbox):
     * counts_per_rev = enc_increments / enc_motor_revs. RW; the
     * diagnostic also uses it to scale inc <-> SI. */
    uint32_t enc_increments     = 16384; /**< 0x608F:1                     */
    uint32_t enc_motor_revs     = 1;     /**< 0x608F:2                     */

    /* Gear ratio (0x6091 record). gear_ratio = motor_revs / shaft_revs;
     * persisted in the app_params blob via the 0x6091 OD hook. */
    uint32_t gear_ratio_motor_revs = 1;  /**< 0x6091:1                     */
    uint32_t gear_ratio_shaft_revs = 1;  /**< 0x6091:2                     */

    /* Protection (vendor 0x2050 record). Stall current crossing held for
     * stall_time trips a fault. */
    uint32_t stall_current      = 0;     /**< 0x2050:1 mA  */
    uint32_t stall_time         = 0;     /**< 0x2050:2 ms  */

    /* ---- Manufacturer-range parameters (0x20xx) -------------------- */
    /* Node ID (vendor-defined; 0x2000:01, uint8). Change with care —
     * commits to the slave's NVM only after a save+reset. */
    uint8_t  node_id            = 0;

    /* Per-phase current-sensor calibration (RW). Offsets 0x2040, gains
     * 0x2041 (counts->A scale). */
    float    current_offset_a   = 0.0f;  /**< 0x2040:1 */
    float    current_offset_b   = 0.0f;  /**< 0x2040:2 */
    float    current_offset_c   = 0.0f;  /**< 0x2040:3 */
    float    current_gain_a     = 1.0f;  /**< 0x2041:1 */
    float    current_gain_b     = 1.0f;  /**< 0x2041:2 */
    float    current_gain_c     = 1.0f;  /**< 0x2041:3 */

    /* Hall commutation alignment offset (0x2060, rad). */
    float    hall_offset        = 0.0f;  /**< 0x2060 */
    /* Motor profile electrical record (0x2070) is written by the Motor
     * Profile editor (writeMotorProfile), not this batch. */
};

/** Identity / device-info readout (CiA-301 0x1000/0x1008/9/A/0x1018). */
struct DeviceInfo
{
    uint32_t device_type   = 0;   /**< 0x1000        */
    QString  device_name;          /**< 0x1008        */
    QString  hw_version;           /**< 0x1009        */
    QString  sw_version;           /**< 0x100A        */
    uint32_t vendor_id     = 0;   /**< 0x1018:01     */
    uint32_t product_code  = 0;   /**< 0x1018:02     */
    uint32_t revision      = 0;   /**< 0x1018:03     */
    uint32_t serial        = 0;   /**< 0x1018:04     */
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
    /** Quick Stop: write controlword 0x0002 (clear bit 2 = trigger
     *  controlled decel per `Quick stop deceleration (0x6085)`).
     *  Different from disableOne (motors free) and from E-STOP
     *  (panic kill). */
    void quickStopOne(int idx);
    /** Manual CiA-402 walk: latch an explicit controlword into the PDO
     *  cycle cache (bypassing cia402_master). The cycle holds the value
     *  on every subsequent RPDO until the next walk step or normal
     *  action overwrites it. Debug aid. */
    void walkControlwordOne(int idx, uint16_t cw);
    void faultReset (int idx);
    /** Brake (CiA-402 Halt bit). Engaged = controlword bit 8 set →
     *  motor decelerates per its mode and holds zero motion; drive
     *  stays in OPERATION_ENABLED. Released = bit 8 cleared → motor
     *  resumes tracking the cached setpoint. */
    void setBrake   (int idx, bool engaged);
    void setMode    (int idx, vrmc::Mode mode);
    void setTarget  (int idx, vrmc::TargetKind which, float value);
    void setId      (int idx, uint8_t newId);

    /* Open-loop V/f. @p engine 0 = FOC (mode 0x6060=-2 + setpoint 0x2031:1
     * freq [mHz] / :2 voltage [mV]; drive must be Enabled), 1 = BSP raw
     * generator (0x2032:1 enable / :2 freq [mHz] / :3 amp [milli-pu]; the
     * drive auto-disables FOC). @p level is volts (FOC) or per-unit (BSP).
     * start writes setpoint + starts; setVfSetpoint streams it live;
     * stop halts that engine. */
    void startVfOpenLoop(int idx, int engine, double freqHz, double level);
    void setVfSetpoint  (int idx, int engine, double freqHz, double level);
    void stopVfOpenLoop (int idx, int engine);

    /* Motor-profile-driven wire scaling. counts_per_rev (from 0x608F /
     * the profile) converts position & velocity inc<->SI; rated torque
     * (Nm) and rated current (A) convert the per-mille torque & current
     * fields. Applied to every slave's control-panel targets + telemetry. */
    void setScaling (double countsPerRev, double ratedNm, double ratedA);

    /* Emergency stop: disable every registered slave at once. */
    void disableAll ();

    /* Gain editor I/O. Response lands on @c gainRead signal. */
    void readGain   (int idx, vrmc::Loop loop);
    void writeGain  (int idx, vrmc::Loop loop, float kp, float ki);
    /** Model-based PI auto-tune at @p bw_hz. Wraps SDK
     *  @c motor_drive_intf_tune_bw via OD 0x2080. Synchronous on the
     *  worker thread; emits @ref gainTuned with the result Kp + Ki when
     *  the slave finishes (or with ok=false on error). */
    void tuneGain   (int idx, vrmc::Loop loop, float bw_hz);
    /** Arm a step-response capture from the board's STEP_RP_EN harness
     *  (OD 0x2080:07..0E). Writes ref_default (:0E) + amp (:09) then
     *  fires trigger (:07); polls :08 status until done, then reads
     *  256 samples per buffer via the indexed read window :0B/:0C/:0D.
     *  Emits @ref stepCaptured. @p ref_default is the baseline reference
     *  held before/after the step (0 = step from rest). */
    void captureStep(int idx, vrmc::Loop loop, float amp, float ref_default);

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

    /** Write a motor profile to the drive: electrical record 0x2070:1..8
     *  + rated torque (0x6076) + rated current (0x6075). Fired by the
     *  Motor Profile editor's OK. */
    void writeMotorProfile(int idx, vrmc::MotorParams mp);

    /** Read the motor profile back from the drive: 0x2070:1..10 (incl. Kt
     *  :9 and encoder CPR :10) + rated current/torque (0x6075/0x6076).
     *  Emits @ref motorProfileRead. CPR is best-effort (encoder variant only). */
    void readMotorProfile(int idx);

    /** Read CiA-301 identity objects (0x1000/0x1008/9/A/0x1018) and emit
     *  @ref deviceInfoRead. Numeric fields are reliable; the strings are
     *  best-effort (skipped if the drive doesn't expose them). */
    void readDeviceInfo  (int idx);

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
    /** Fired by @ref tuneGain when the slave finishes (or errors out). */
    void gainTuned(int idx, vrmc::Loop loop, float kp, float ki, bool ok);
    /** Fired by @ref captureStep with the two 256-sample buffers + sample
     *  rate (Hz). On failure, buf0/buf1 are empty and ok = false. */
    void stepCaptured(int idx, vrmc::Loop loop,
                       QVector<float> buf0, QVector<float> buf1,
                       float sample_rate_hz, bool ok);
    void generatorStarted(int idx);
    void generatorStopped(int idx);
    void pdoMappingApplied(int idx, bool isTpdo, bool ok, QString message);
    void driveConfigRead   (int idx, vrmc::DriveConfig cfg, bool ok,
                            QString message);
    void driveConfigWritten(int idx, bool ok, QString message);
    /** Result of @ref readMotorProfile: the slave's motor profile (incl.
     *  Kt + CPR). Drives the Motor Profile editor/view's "load from slave". */
    void motorProfileRead  (int idx, vrmc::MotorParams mp, bool ok,
                            QString message);
    void deviceInfoRead    (int idx, vrmc::DeviceInfo info, bool ok,
                            QString message);
    /** Encoder-zero / torque-tare completion with the raw value that was
     *  captured (and subsequently written). Dialog uses this to confirm
     *  the action rather than waiting on the next SDO refresh. */
    void calibrationDone   (int idx, QString what, int64_t raw, bool ok,
                            QString message);
    /** Master-side controlword update after arm/disarm/quick-stop.
     *  JointControlPanel mirrors this in its readout strip so the
     *  operator sees what the master is actually telling each slave. */
    void controlwordCached (int idx, uint16_t cw);

    /** Custom-SDO operation completed. @p valueDecoded is the read-back
     *  bytes formatted as a little-endian unsigned hex string (empty
     *  for writes or failed reads). @p ok plus @p message give the
     *  dialog enough to render success/failure with the abort code. */
    void customSdoDone(int slaveIdx, bool isWrite, uint16_t odIdx, uint8_t sub,
                       bool ok, QString valueDecoded, QString message);

public slots:
    /** Run an expedited SDO upload (read). @p byteLen is the number of
     *  bytes to fetch (1/2/4 for typical CiA-301 datatypes). Result is
     *  fired back as @ref customSdoDone. */
    void customSdoRead (int slaveIdx, uint16_t odIdx, uint8_t sub, int byteLen);
    /** Run an expedited SDO download (write). @p bytes is the LE-encoded
     *  payload; len matches the underlying datatype size. */
    void customSdoWrite(int slaveIdx, uint16_t odIdx, uint8_t sub, QByteArray bytes);

private slots:
    void onTick();                  /**< UI refresh + cached PDO snapshot read */
    void onCycleTick();             /**< PDO cycle: send 1 RPDO per slot     */
    void onGenTick();               /**< QTimer callback pushing waveform value */

private:
    void teardown();

    CanBackend                  m_can;
    master_mgr_t*               m_mgr      = nullptr;
    /* Two-rate timers, matching motor_drive_master.c's split between
     * cycle_run_loop (high-rate RPDO burst + transport pump) and the
     * shell/UI (low-rate user interaction). The cycle timer holds the
     * same mutex as bringup/disable/quick-stop so SDO walks never race
     * with cached-controlword RPDOs. */
    QTimer*                     m_tick     = nullptr;   /* UI refresh @ m_hz */
    QTimer*                     m_cycle    = nullptr;   /* PDO cycle @ m_cycleHz */
    /* Two-rate clock: PDO cycle pushed as high as the wire + vendor
     * USB stack tolerate, UI refresh kept slow so the snapshot signal
     * queue can never back up on the UI thread.
     *
     *   m_cycleHz = 1000  → 1 ms RPDO period.
     *     Matches motor_drive_master --cycle 1000. Bode sweeps + step
     *     response can analyse out to ~500 Hz. Per-slot wire load at
     *     500 k arb / 2 M data ≈ 8 % bus, so 5+ slots still fit
     *     comfortably. Vendor libcontrolcanfd USB poll thread sits at
     *     ~20-25 % CPU on a typical laptop — that's the explicit
     *     tradeoff for high cycle.
     *
     *   m_hz = 20         → 50 ms UI refresh period.
     *     Decoupled from cycle so the snapshot signal (emitted via
     *     QueuedConnection, never coalesced) doesn't pile up on the UI
     *     thread regardless of how fast cycle runs. Buttons feel
     *     instant; chart is OpenGL-accelerated so 9 series × 60 s
     *     buffer paints in ~1 ms. */
    int                         m_hz       = 20;
    int                         m_cycleHz  = 1000;
    int                         m_refreshTick = 0;      /* SDO housekeeping divider */
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

    /* Wire scaling (motor-profile-driven; see setScaling). Defaults match
     * the board's 14-bit encoder + 0.5 Nm rated until a profile loads. */
    uint32_t                    m_countsPerRev = 16384u;
    float                       m_ratedNm      = 0.5f;
    float                       m_ratedA       = 2.0f;
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
Q_DECLARE_METATYPE(vrmc::DeviceInfo)
Q_DECLARE_METATYPE(QVector<float>)
Q_DECLARE_METATYPE(vrmc::MotorParams)
