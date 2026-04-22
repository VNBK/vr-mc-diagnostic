/**
 * @file   PdoMappingView.hpp
 * @brief  Read-only inspector for the TPDO1 / RPDO1 mappings.
 *
 * The mapping itself is fixed in V1 (matches the CiA 402 drive profile
 * used by the simulator): TPDO1 = Statusword + PositionActual +
 * VelocityActual + TorqueActual + ErrorCode, RPDO1 = Controlword +
 * TargetPosition + TargetVelocity + TargetTorque.
 *
 * The view decorates each entry with the live value pulled from the
 * latest @ref SlaveSnapshot, so the operator can watch the PDO payload
 * tick and confirm the diagnostic is actually consuming PDO — not SDO
 * poll — for telemetry.
 */

#pragma once

#include "MasterWorker.hpp"

#include <QWidget>

class QLabel;
class QTableWidget;

namespace vrmc {

class PdoMappingView : public QWidget
{
    Q_OBJECT
public:
    explicit PdoMappingView(QWidget* parent = nullptr);

public slots:
    void setActiveSlave(int idx, const QString& name = QString());
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);

signals:
    /**
     * @brief User confirmed new mapping; MainWindow should forward to
     *        the worker for the actual SDO push.
     */
    void applyRequested(int idx, bool isTpdo,
                        QVector<vrmc::CanBackend::PdoMapEntry> entries);

private slots:
    void onEditTpdo();
    void onEditRpdo();

private:
    void         buildTpdoTable();
    void         buildRpdoTable();
    void         refreshHeaders(const SlaveSnapshot* s);
    void         refreshValues (const SlaveSnapshot* s);
    QTableWidget* makeTable(int rows);

    int           m_activeIdx = -1;
    QString       m_activeName;

    QLabel*       m_tpdoHeader = nullptr;
    QLabel*       m_rpdoHeader = nullptr;
    QTableWidget* m_tpdo       = nullptr;
    QTableWidget* m_rpdo       = nullptr;

    /* Current mapping, kept so the editor can open with a seeded list.
     * Default values match the CiA 402 layout applied by the sim; if a
     * user successfully pushes a new mapping these get updated. */
    QVector<vrmc::CanBackend::PdoMapEntry> m_tpdoEntries;
    QVector<vrmc::CanBackend::PdoMapEntry> m_rpdoEntries;
};

}  // namespace vrmc
