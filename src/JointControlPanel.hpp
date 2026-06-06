/**
 * @file   JointControlPanel.hpp
 * @brief  Per-slave Control tab: power buttons, mode, setpoint, and a
 *         live state/controlword readout strip.
 *
 * The panel caches the most recent SlaveSnapshot for the active slave
 * (via @ref onSnapshots) so the readout strip and the state-aware
 * button enable/disable logic don't have to re-fetch on every update.
 * The cached @em controlword shown in the strip is the value the
 * master is currently streaming via the PDO keep-alive — surfaced
 * here through @ref onControlwordCached so the operator can see
 * "master commanding 0x000F, slave reports OPERATION_ENABLED".
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;

namespace vrmc {

class JointControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit JointControlPanel(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

public slots:
    /** Receive the per-tick snapshot stream so the readout strip and
     *  state-aware buttons stay live. Filters internally to the active
     *  slave. */
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);

    /** Notify the panel of the master's currently-streamed controlword
     *  for a given slave so the readout strip can show it. Emitted by
     *  MasterWorker after arm/disarm/quick-stop. */
    void onControlwordCached(int idx, uint16_t cw);

signals:
    /* Routed by MainWindow to MasterWorker via QueuedConnection. */
    void bringupRequested  (int idx, uint32_t timeoutMs);
    void enableRequested   (int idx);
    void disableRequested  (int idx);
    void quickStopRequested(int idx);
    void faultResetRequested(int idx);
    /** Brake toggle. @p engaged = true → CiA-402 Halt bit set in cycle
     *  controlword (motor freezes, stays in OPERATION_ENABLED). */
    void brakeRequested    (int idx, bool engaged);
    void modeRequested     (int idx, vrmc::Mode mode);
    void targetRequested   (int idx, vrmc::TargetKind which, float value);
    /** Manual CiA-402 walk: send one explicit controlword via PDO,
     *  bypassing the cia402_master state machine. Debug aid only. */
    void walkControlwordRequested(int idx, uint16_t cw);
    /** Open-loop V/f. @p engine 0 = FOC (unified 0x2030 type=2 +
     *  0x6060=-2, needs Enable), 1 = BSP raw generator (0x2030 type=1,
     *  auto-disables FOC). @p level
     *  is volts (FOC) or per-unit amplitude (BSP). Start writes setpoint +
     *  starts; live edits stream the setpoint; Stop halts that engine. */
    void vfStartRequested  (int idx, int engine, double freqHz, double level);
    void vfSetpointChanged (int idx, int engine, double freqHz, double level);
    void vfStopRequested    (int idx, int engine);

private slots:
    void emitTarget();

private:
    /** Refresh the readout strip from @c m_snap + @c m_cwCached. */
    void refreshReadout();
    /** Enable/disable individual buttons based on the cached CiA-402
     *  state so the operator can't click Bringup when already enabled,
     *  Enable while in FAULT, etc. */
    void refreshButtons();

    /** @brief Decode CiA-402 statusword bits → state name.
     *  @return One of NOT_READY / SWITCH_ON_DISABLED / READY /
     *          SWITCHED_ON / OPERATION_ENABLED / QUICK_STOP_ACTIVE /
     *          FAULT_REACTION / FAULT / UNKNOWN. */
    static QString stateName(uint16_t statusword);

    /** @brief Categorise statusword into a coarse enum we can switch
     *  on for button enabling. Returns the numeric value of the bits
     *  pattern (0 = NOT_READY, 1 = SWITCH_ON_DISABLED, 2 = READY,
     *  3 = SWITCHED_ON, 4 = OPERATION_ENABLED, 5 = QUICK_STOP_ACTIVE,
     *  6 = FAULT_REACTION, 7 = FAULT, 99 = UNKNOWN). */
    static int stateCode(uint16_t statusword);

    int            m_idx = -1;
    SlaveSnapshot  m_snap;            /**< cached snapshot of m_idx     */
    uint16_t       m_cwCached = 0;    /**< controlword master streams   */
    bool           m_hasSnap   = false;
    /** Cache the last state-code we ran @ref refreshButtons against so
     *  onSnapshots can skip rebuilding the enable/disable matrix when
     *  nothing has changed. Without this, 30 Hz snapshots × ~20
     *  setEnabled calls = sustained UI-thread paint thrash. */
    int            m_lastBtnState = -2;
    bool           m_lastBtnPdoFresh = false;
    /** Cache for setStyleSheet gating on the state badge + error label.
     *  setStyleSheet reparses CSS and triggers a polish event; calling
     *  it every snapshot pegs the UI thread at the refresh rate. */
    int            m_lastReadoutState   = -1;
    bool           m_lastErrorVisible   = false;

    QLabel*        m_label        = nullptr;
    QLabel*        m_state        = nullptr;
    QLabel*        m_cwLabel      = nullptr;
    QLabel*        m_modeLabel    = nullptr;
    QPushButton*   m_bringup      = nullptr;
    QPushButton*   m_enable       = nullptr;
    QPushButton*   m_disable      = nullptr;
    QPushButton*   m_quickStop    = nullptr;
    QPushButton*   m_faultReset   = nullptr;
    QPushButton*   m_brake        = nullptr;   /**< checkable, CiA-402 Halt */
    QComboBox*     m_modeCombo    = nullptr;
    QComboBox*     m_targetCombo  = nullptr;
    QDoubleSpinBox*m_valueSpin    = nullptr;
    QPushButton*   m_sendBtn      = nullptr;

    /* Open-loop V/f group. m_vfLevel is volts (FOC) or per-unit (BSP); the
     * label/range retitle on engine change. */
    QComboBox*      m_vfEngine    = nullptr;
    QDoubleSpinBox* m_vfFreq      = nullptr;
    QDoubleSpinBox* m_vfLevel     = nullptr;
    QLabel*         m_vfLevelLabel = nullptr;
    QPushButton*    m_vfStart     = nullptr;
    QPushButton*    m_vfStop      = nullptr;
    bool            m_vfRunning   = false;

    /** Readout-strip checkbox: when checked, the tracking-error label
     *  prints Δq in degrees; otherwise in radians. Only affects
     *  display — internal units stay rad / rad·s⁻¹ everywhere. */
    QCheckBox*     m_degToggle    = nullptr;

    /* Phase B widgets: presets / jog / mode-aware unit. */
    QPushButton*   m_preset0      = nullptr;
    QPushButton*   m_presetQp     = nullptr;   /* +π/4 */
    QPushButton*   m_presetQn     = nullptr;   /* -π/4 */
    QPushButton*   m_presetHome   = nullptr;
    QPushButton*   m_jogMinus     = nullptr;
    QPushButton*   m_jogPlus      = nullptr;
    QDoubleSpinBox*m_jogStep      = nullptr;

    /** Send `value` as the currently-selected target kind without
     *  touching the spinbox display (presets / jog use this). */
    void sendTarget(double value);
    /** Reflect a Mode change into the target combo + spinbox suffix
     *  so unit labelling stays consistent. */
    void syncTargetKindToMode();

    /** @brief Decode a CiA-402 error_code into "0xCODE name". Empty
     *  for code == 0. Covers the common DS-402 §8.2 codes; falls back
     *  to "0xCODE (manufacturer)" for unknown codes. */
    static QString decodeErrorCode(uint16_t code);

    /** @brief Recompute the slider's range from the currently-selected
     *  target kind so the operator can sweep useful values (e.g. ±π
     *  for Position, ±10 rad/s for Velocity). */
    void resyncSliderRange();

    /** @return true iff the user has the "deg" checkbox on AND the
     *  currently-selected target kind is Position (the only kind for
     *  which radians ↔ degrees makes sense). */
    bool   isDegMode() const;
    /** @return Multiplier from rad to current display unit
     *  (`180/π` in deg mode, `1.0` otherwise). Used to map preset
     *  rad values into the displayed unit and to convert spinbox
     *  reads back to rad before they're emitted on the wire. */
    double displayFactor() const;
    /** @brief Convert spinbox value <-> slider in/out of deg mode and
     *  update suffix + slider range. Called from the deg-toggle slot. */
    void onDegToggled(bool checked);
    /** @brief Bidirectional sync between spinbox + slider without
     *  triggering infinite signal loops. */
    void onSpinboxChanged(double v);
    void onSliderChanged(int v);
    /** @brief Load / save the named-presets JSON store. */
    void loadPresetStore();
    void savePresetStore();
    /** @brief Path of the JSON store (~/.config/vrmc/control_presets.json). */
    static QString presetStorePath();

    /* Phase C widgets. */
    QLabel*        m_errorLabel       = nullptr;   /* decoded error code   */
    QLabel*        m_trackLabel       = nullptr;   /* tracking error Δq    */
    QSlider*       m_setpointSlider   = nullptr;
    QCheckBox*     m_liveStream       = nullptr;
    QPushButton*   m_walkShutdown     = nullptr;
    QPushButton*   m_walkSwitchOn     = nullptr;
    QPushButton*   m_walkEnableOp     = nullptr;
    QPushButton*   m_walkDisableV     = nullptr;
    QComboBox*     m_presetCombo      = nullptr;
    QPushButton*   m_presetSaveBtn    = nullptr;
    QPushButton*   m_presetDelBtn     = nullptr;
    bool           m_syncingFromSlider = false;
    bool           m_syncingFromSpin   = false;
};

}  // namespace vrmc
