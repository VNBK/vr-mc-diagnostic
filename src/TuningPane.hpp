/**
 * @file   TuningPane.hpp
 * @brief  Tuning tools — step response + frequency (Bode) response.
 *
 * Drives the existing MasterWorker signal generator, listens to the
 * snapshot stream, and plots:
 *   - StepResponseView: overlay of commanded step vs actual,
 *     with overshoot / rise / settling-time metrics.
 *   - BodeView:          sine sweep, sine-correlation extracts
 *     |H(f)| (dB) and phase (deg) at each test frequency; two log-axis
 *     charts.
 *
 * Widgets are self-contained — they own the capture buffers and the
 * per-test state machine, and emit @ref startGenRequested /
 * @ref stopGenRequested so MainWindow can forward to the worker.
 */

#pragma once

#include "MasterWorker.hpp"
#include "MotorProfile.hpp"

#include <QElapsedTimer>
#include <QVector>
#include <QWidget>

QT_FORWARD_DECLARE_CLASS(QChart)
QT_FORWARD_DECLARE_CLASS(QChartView)
QT_FORWARD_DECLARE_CLASS(QComboBox)
QT_FORWARD_DECLARE_CLASS(QDoubleSpinBox)
QT_FORWARD_DECLARE_CLASS(QLabel)
QT_FORWARD_DECLARE_CLASS(QLineSeries)
QT_FORWARD_DECLARE_CLASS(QLogValueAxis)
QT_FORWARD_DECLARE_CLASS(QPushButton)
QT_FORWARD_DECLARE_CLASS(QScatterSeries)
QT_FORWARD_DECLARE_CLASS(QSpinBox)
QT_FORWARD_DECLARE_CLASS(QTabWidget)
QT_FORWARD_DECLARE_CLASS(QValueAxis)

namespace vrmc {

/* ======================================================================
 * Step response tuning view.
 * ====================================================================== */

class StepResponseView : public QWidget
{
    Q_OBJECT
public:
    explicit StepResponseView(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

signals:
    void startGenRequested(int idx, vrmc::GenCfg cfg);
    void stopGenRequested (int idx);

public slots:
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);
    void onGeneratorStarted(int idx);
    void onGeneratorStopped(int idx);

private slots:
    void onRunClicked();

private:
    void computeAndShowMetrics();
    void resetChart();

    int                 m_idx       = -1;
    bool                m_capturing = false;
    QElapsedTimer       m_t0;

    /* UI */
    QLabel*             m_label      = nullptr;
    QComboBox*          m_target     = nullptr;
    QDoubleSpinBox*     m_amp        = nullptr;
    QDoubleSpinBox*     m_offset     = nullptr;
    QDoubleSpinBox*     m_duration   = nullptr;
    QPushButton*        m_runBtn     = nullptr;
    QLabel*             m_metrics    = nullptr;

    /* Plot */
    QChart*             m_chart    = nullptr;
    QLineSeries*        m_cmdLine  = nullptr;
    QLineSeries*        m_actual   = nullptr;
    QValueAxis*         m_axisX    = nullptr;
    QValueAxis*         m_axisY    = nullptr;

    /* Capture. (t_s, cmd, actual) */
    struct Sample { double t; double cmd; double actual; };
    QVector<Sample>     m_samples;
    double              m_stepBase = 0.0;
    double              m_stepMag  = 1.0;
    double              m_duration_s = 2.0;
};


/* ======================================================================
 * Bode / frequency response view.
 * ====================================================================== */

class BodeView : public QWidget
{
    Q_OBJECT
public:
    explicit BodeView(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

signals:
    void startGenRequested(int idx, vrmc::GenCfg cfg);
    void stopGenRequested (int idx);

public slots:
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);
    void onGeneratorStarted(int idx);
    void onGeneratorStopped(int idx);

private slots:
    void onRunClicked();

private:
    void startNextPoint();
    void analyseCurrentPoint();
    void resetCharts();

    int                 m_idx       = -1;
    bool                m_sweeping  = false;
    int                 m_pointIdx  = 0;
    QVector<double>     m_freqs;
    double              m_amp       = 0.5;
    double              m_pointDur  = 1.0;   /* seconds per point */
    double              m_captureStart = 0.0;
    QElapsedTimer       m_t0;

    struct Sample { double t; double actual; };
    QVector<Sample>     m_samples;

    /* UI */
    QLabel*             m_label      = nullptr;
    QComboBox*          m_target     = nullptr;
    QDoubleSpinBox*     m_fStart     = nullptr;
    QDoubleSpinBox*     m_fEnd       = nullptr;
    QSpinBox*           m_points     = nullptr;
    QDoubleSpinBox*     m_ampSpin    = nullptr;
    QSpinBox*           m_cycles     = nullptr;
    QPushButton*        m_runBtn     = nullptr;
    QLabel*             m_status     = nullptr;

    /* Plots. Two stacked charts: magnitude (dB), phase (deg). */
    QChart*             m_magChart   = nullptr;
    QLineSeries*        m_magLine    = nullptr;
    QScatterSeries*     m_magDots    = nullptr;
    QLogValueAxis*      m_magAxisX   = nullptr;
    QValueAxis*         m_magAxisY   = nullptr;

    QChart*             m_phChart    = nullptr;
    QLineSeries*        m_phLine     = nullptr;
    QScatterSeries*     m_phDots     = nullptr;
    QLogValueAxis*      m_phAxisX    = nullptr;
    QValueAxis*         m_phAxisY    = nullptr;
};


/* ======================================================================
 * Auto-tune view: model-based PI tune via OD 0x2080 + optional follow-up
 * step-response capture from the board's STEP_RP_EN harness (no signal
 * generator -- the board fills bufferData[] itself and the master reads
 * it back as DOMAIN).
 * ====================================================================== */

class AutoTuneView : public QWidget
{
    Q_OBJECT
public:
    explicit AutoTuneView(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

    /** Hand the latest motor name-plate down so the BW spinbox can be
     *  back-computed from the readback Kp via @ref bwFromKp. Called by
     *  MainWindow after every successful @c readMotorProfile / editor
     *  commit. Triggers an immediate refresh if Kp is already populated. */
    void setMotorParams(const vrmc::MotorParams& p);

signals:
    /** Request a model-based PI auto-tune on the slave. Result lands on
     *  @ref onGainTuned. */
    void tuneRequested(int idx, vrmc::Loop loop, float bw_hz);
    /** Request a step-response capture from the board's STEP_RP_EN harness.
     *  @p ref_default sets the baseline reference held before/after the
     *  step (0 = step from rest). Result lands on @ref onStepCaptured. */
    void captureStepRequested(int idx, vrmc::Loop loop, float amp, float ref_default);
    /** Refresh the displayed Kp/Ki by reading 0x60F6/F9/FB from the slave.
     *  Fired on slave-select, loop combo change, and dock re-open so the
     *  view reflects what the live PI is using. Reply -> @ref onGainRead. */
    void readGainRequested(int idx, vrmc::Loop loop);

public slots:
    /** Wired to @c MasterWorker::gainTuned. */
    void onGainTuned   (int idx, vrmc::Loop loop, float kp, float ki, bool ok);
    /** Wired to @c MasterWorker::stepCaptured. */
    void onStepCaptured(int idx, vrmc::Loop loop,
                         QVector<float> buf0, QVector<float> buf1,
                         float sample_rate_hz, bool ok);
    /** Wired to @c MasterWorker::gainRead. Updates the result Kp/Ki view
     *  (independent from the Tune button -- this is the OD readback path). */
    void onGainRead    (int idx, vrmc::Loop loop, float kp, float ki, bool ok);

private slots:
    void onTuneClicked();
    void onCaptureClicked();
    void onLoopChanged(int idx);

private:
    void replotStep(const QVector<float>& buf0, const QVector<float>& buf1,
                    float sample_rate_hz);
    void resetChart();

    int                 m_idx = -1;

    /* Cached motor name-plate, pushed from MainWindow. Drives BW
     * back-compute on Kp readback (board doesn't persist BW). */
    MotorParams         m_motorParams;

    /* UI */
    QLabel*             m_label      = nullptr;
    QComboBox*          m_loop       = nullptr;
    QDoubleSpinBox*     m_bw         = nullptr;
    QPushButton*        m_tuneBtn    = nullptr;
    QLabel*             m_tuneStatus = nullptr;
    QDoubleSpinBox*     m_kpView     = nullptr;
    QDoubleSpinBox*     m_kiView     = nullptr;

    QDoubleSpinBox*     m_amp        = nullptr;
    QDoubleSpinBox*     m_refDefault = nullptr;
    QPushButton*        m_captureBtn = nullptr;
    QLabel*             m_captureStatus = nullptr;

    /* Plot. buffer0 = measured signal (Iq or omega depending on loop);
     * buffer1 = reference / control effort. */
    QChart*             m_chart      = nullptr;
    QLineSeries*        m_meas       = nullptr;
    QLineSeries*        m_ref        = nullptr;
    QValueAxis*         m_axisX      = nullptr;
    QValueAxis*         m_axisY      = nullptr;
};


/* ======================================================================
 * Container widget with Step + Bode + Auto-Tune as tabs.
 * ====================================================================== */

class TuningPane : public QWidget
{
    Q_OBJECT
public:
    explicit TuningPane(QWidget* parent = nullptr);

    StepResponseView* step()     const { return m_step;     }
    BodeView*         bode()     const { return m_bode;     }
    AutoTuneView*     autoTune() const { return m_autoTune; }

    void setActiveSlave(int idx, const QString& name = {});

private:
    QTabWidget*       m_tabs     = nullptr;
    StepResponseView* m_step     = nullptr;
    BodeView*         m_bode     = nullptr;
    AutoTuneView*     m_autoTune = nullptr;
};

}  // namespace vrmc
