#include "SlaveTableModel.hpp"

extern "C" {
#include "motor_drive_interface.h"
}

namespace vrmc {

SlaveTableModel::SlaveTableModel(QObject* parent)
    : QAbstractTableModel(parent) {}

int SlaveTableModel::rowCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : m_rows.size();
}

int SlaveTableModel::columnCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : ColCount;
}

QString SlaveTableModel::stateToString(int state)
{
    switch (state){
    case MOTOR_INTF_STATE_OFFLINE:  return QStringLiteral("offline");
    case MOTOR_INTF_STATE_DISABLED: return QStringLiteral("disabled");
    case MOTOR_INTF_STATE_READY:    return QStringLiteral("ready");
    case MOTOR_INTF_STATE_ENABLED:  return QStringLiteral("enabled");
    case MOTOR_INTF_STATE_FAULT:    return QStringLiteral("fault");
    default:                        return QStringLiteral("?");
    }
}

QVariant SlaveTableModel::data(const QModelIndex& idx, int role) const
{
    if (!idx.isValid() || idx.row() >= m_rows.size()){ return {}; }
    if (role != Qt::DisplayRole && role != Qt::ToolTipRole){ return {}; }
    const auto& s = m_rows.at(idx.row());
    switch (idx.column()){
    case ColIdx:      return s.idx;
    case ColId:       return s.id;
    case ColName:     return s.name;
    case ColState:    return stateToString(s.state);
    case ColOnline:   return s.online ? QStringLiteral("yes") : QStringLiteral("no");
    case ColPosition: return QString::number(s.position, 'f', 4);
    case ColVelocity: return QString::number(s.velocity, 'f', 4);
    case ColTorque:   return QString::number(s.torque,   'f', 4);
    case ColCurrent:  return QString::number(s.current,     'f', 3);
    case ColTemperature: return QString::number(s.temperature, 'f', 1);
    case ColError:
        /* Hide stale 0x0000 unless PDO is alive; otherwise the column
         * implies "no fault" even when the wire hasn't actually
         * reported. Once a TPDO has landed (pdoFresh) the 0 means
         * "drive is reporting clean". */
        if (!s.pdoFresh){ return QStringLiteral("—"); }
        return QStringLiteral("0x%1").arg(s.errorCode, 4, 16, QChar('0'));
    }
    return {};
}

QVariant SlaveTableModel::headerData(int section, Qt::Orientation o, int role) const
{
    if (role != Qt::DisplayRole){ return {}; }
    if (o == Qt::Vertical){ return section; }
    switch (section){
    case ColIdx:      return QStringLiteral("idx");
    case ColId:       return QStringLiteral("id");
    case ColName:     return QStringLiteral("name");
    case ColState:    return QStringLiteral("state");
    case ColOnline:   return QStringLiteral("online");
    case ColPosition: return QStringLiteral("pos (rad)");
    case ColVelocity: return QStringLiteral("vel (rad/s)");
    case ColTorque:   return QStringLiteral("trq (Nm)");
    case ColCurrent:    return QStringLiteral("I (A)");
    case ColTemperature:return QStringLiteral("T (°C)");
    case ColError:      return QStringLiteral("err");
    }
    return {};
}

void SlaveTableModel::update(const QVector<SlaveSnapshot>& snaps)
{
    if (snaps.size() != m_rows.size()){
        beginResetModel();
        m_rows = snaps;
        endResetModel();
        return;
    }
    m_rows = snaps;
    emit dataChanged(index(0, 0),
                     index(m_rows.size() - 1, ColCount - 1));
}

}  // namespace vrmc
