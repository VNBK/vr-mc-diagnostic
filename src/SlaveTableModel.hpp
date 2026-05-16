/**
 * @file   SlaveTableModel.hpp
 * @brief  QAbstractTableModel wrapping SlaveSnapshot vectors for QTableView.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QAbstractTableModel>

namespace vrmc {

class SlaveTableModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    enum Column {
        ColIdx = 0,
        ColId,
        ColName,
        ColState,
        ColOnline,
        ColPosition,
        ColVelocity,
        ColTorque,
        ColCurrent,      /**< A                                            */
        ColTemperature,  /**< °C                                           */
        ColError,        /**< CiA-402 error_code (0x603F), PDO-fresh only  */
        ColCount,
    };

    explicit SlaveTableModel(QObject* parent = nullptr);

    int rowCount    (const QModelIndex& parent = QModelIndex()) const override;
    int columnCount (const QModelIndex& parent = QModelIndex()) const override;
    QVariant data       (const QModelIndex& idx, int role = Qt::DisplayRole) const override;
    QVariant headerData (int section, Qt::Orientation o, int role = Qt::DisplayRole) const override;

public slots:
    void update(const QVector<vrmc::SlaveSnapshot>& snaps);

private:
    static QString stateToString(int state);

    QVector<SlaveSnapshot> m_rows;
};

}  // namespace vrmc
