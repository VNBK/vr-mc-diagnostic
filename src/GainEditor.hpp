/**
 * @file   GainEditor.hpp
 * @brief  Per-loop Kp/Ki spinbox grid + Read / Apply for one slave.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>

class QDoubleSpinBox;
class QPushButton;

namespace vrmc {

class GainEditor : public QWidget
{
    Q_OBJECT
public:
    explicit GainEditor(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

signals:
    /** Request a refresh of the chosen loop; reply arrives via @c onGainRead. */
    void readGainRequested (int idx, vrmc::Loop loop);
    /** Commit Kp/Ki for the chosen loop. */
    void writeGainRequested(int idx, vrmc::Loop loop, float kp, float ki);

public slots:
    /** Slot wired to @ref MasterWorker::gainRead. */
    void onGainRead(int idx, vrmc::Loop loop, float kp, float ki, bool ok);

private:
    struct Row {
        Loop             loop;
        QDoubleSpinBox*  kp   = nullptr;
        QDoubleSpinBox*  ki   = nullptr;
        QPushButton*     read = nullptr;
        QPushButton*     apply = nullptr;
    };

    void emitRead (Loop loop);
    void emitApply(Loop loop);

    int           m_idx = -1;
    QVector<Row>  m_rows;
};

}  // namespace vrmc
