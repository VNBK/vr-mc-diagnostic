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

    /* Protection (0x6065 / 0x6066 / 0x607D / 0x6080 / 0x6072 / 0x6076). */
    uint32_t following_error    = 5000;
    uint16_t following_error_ms = 100;
    int32_t  pos_limit_min      = -2147483647;
    int32_t  pos_limit_max      =  2147483647;
    uint32_t max_motor_speed    = 10000;
    uint16_t max_torque         = 2000;  /**< per-mille of rated  */
    uint32_t rated_torque       = 0;     /**< 0x6076, mNm                  */

    /* Read-only telemetry (0x6078). Populated on Read; ignored on
     * Write. Kept in the DriveConfig blob so dialogs can present a
     * single snapshot. */
    int16_t  current_actual     = 0;     /**< 0x6078, per-mille of rated I */

    /* Encoder / scaling (0x608F:1 / 0x6091 / 0x6092). */
    uint32_t enc_resolution     = 16384; /**< counts per rev       */
    uint32_t gear_num           = 1;
    uint32_t gear_den           = 1;
    uint32_t feed_const_num     = 1;
    uint32_t feed_const_den     = 1;

    /* ---- Manufacturer-range parameters (0x20xx) --------------------
     *
     * Vendor-defined OD entries. Default index choices below assume a
     * typical FOC drive layout; if your hardware uses different
     * indices, override at the Custom-SDO tab or edit driveFields()
     * in MasterWorker.cpp. All reads/writes go via the standard SDO
     * helpers, so a slave that doesn't expose the index just abort
     * with `0x06020000` and the read result is silently 0.
     */
    /* Node ID (vendor-defined; 0x2000:00, uint8). Change with care —
     * commits to the slave's NVM only after a save+reset. */
    uint8_t  node_id            = 0;

    /* Per-loop manufacturer gain (parallel to CiA-402 standard via
     * motor_drive_intf_get/set_gain; the Gains tab uses those, this
     * tab uses vendor entries so you can compare or override). */
    float    manuf_cur_kp       = 0.0f; /**< 0x2010:00 */
    float    manuf_cur_ki       = 0.0f; /**< 0x2011:00 */
    float    manuf_vel_kp       = 0.0f; /**< 0x2020:00 */
    float    manuf_vel_ki       = 0.0f; /**< 0x2021:00 */
    float    manuf_pos_kp       = 0.0f; /**< 0x2030:00 */
    float    manuf_pos_ki       = 0.0f; /**< 0x2031:00 */

    /* Per-phase current-sensor calibration. Offsets are signed raw
     * counts (zero-current ADC reading); gains are unsigned scale
     * factors (ADC counts → Amperes). */
    int16_t  current_offset_a   = 0;    /**< 0x2040:01 */
    int16_t  current_offset_b   = 0;    /**< 0x2040:02 */
    int16_t  current_offset_c   = 0;    /**< 0x2040:03 */
    uint16_t current_gain_a     = 0;    /**< 0x2041:01 */
    uint16_t current_gain_b     = 0;    /**< 0x2041:02 */
    uint16_t current_gain_c     = 0;    /**< 0x2041:03 */

    /* Fault thresholds (all under 0x2050:nn). The drive raises the
     * matching latched-fault bit when the named signal crosses the
     * threshold for the (optional) accompanying time window. Units
     * follow the typical FOC-drive vendor convention noted on each
     * field — adjust if your drive disagrees. */
    uint32_t over_current        = 0;   /**< 0x2050:01  mA          */
    uint16_t over_load           = 0;   /**< 0x2050:02  % of rated  */
    uint32_t over_load_ms        = 0;   /**< 0x2050:03  ms          */
    uint32_t current_loss_phase  = 0;   /**< 0x2050:04  mA threshold (phase missing) */
    uint32_t current_loss_phase_ms = 0; /**< 0x2050:05  ms          */
    uint32_t unbalance_current   = 0;   /**< 0x2050:06  mA delta    */
    uint32_t stall_ms            = 0;   /**< 0x2050:07  ms (no motion under load) */
    uint32_t over_voltage        = 0;   /**< 0x2050:08  mV          */
    uint32_t under_voltage       = 0;   /**< 0x2050:09  mV          */
    int16_t  over_temperature    = 0;   /**< 0x2050:0A  °C × 10     */
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
    /** Manual CiA-402 walk: stream one explicit controlword via PDO
     *  (bypassing cia402_master). Re-arms tx_active so the value
     *  sticks until the next walk step or normal action. Debug aid. */
    void walkControlwordOne(int idx, uint16_t cw);
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
