#include "SlavePickerBar.hpp"
#include "SlaveTableModel.hpp"

#include <QAbstractItemModel>
#include <QComboBox>
#include <QHBoxLayout>
#include <QItemSelectionModel>
#include <QLabel>
#include <QPushButton>

namespace vrmc {

SlavePickerBar::SlavePickerBar(QWidget* parent) : QWidget(parent)
{
    auto* row = new QHBoxLayout(this);
    row->setContentsMargins(8, 4, 8, 4);
    row->setSpacing(12);

    row->addWidget(new QLabel(tr("Slave:"), this));

    m_combo = new QComboBox(this);
    m_combo->setMinimumWidth(220);
    m_combo->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    row->addWidget(m_combo);

    /* Live strip. Each label uses a monospaced font so the values
     * don't jitter horizontally as digits flip. State has its own
     * colour-coded background (green/amber/red) for at-a-glance
     * status. */
    auto makeStrip = [this](){
        auto* l = new QLabel(QStringLiteral("—"), this);
        QFont f = l->font();
        f.setFamily(QStringLiteral("monospace"));
        l->setFont(f);
        return l;
    };
    m_stateLbl = makeStrip();
    m_posLbl   = makeStrip();
    m_velLbl   = makeStrip();
    m_trqLbl   = makeStrip();
    m_errLbl   = makeStrip();
    m_stateLbl->setText(tr("state: —"));
    m_posLbl  ->setText(tr("q   —"));
    m_velLbl  ->setText(tr("ω   —"));
    m_trqLbl  ->setText(tr("τ   —"));
    m_errLbl  ->setText(tr("err —"));

    row->addWidget(m_stateLbl);
    row->addWidget(m_posLbl);
    row->addWidget(m_velLbl);
    row->addWidget(m_trqLbl);
    row->addWidget(m_errLbl);
    row->addStretch(1);

    m_expandBtn = new QPushButton(tr("Expand grid ⤵"), this);
    m_expandBtn->setCheckable(true);
    m_expandBtn->setToolTip(tr("Show the full slave table grid below "
                               "this bar (multi-column view of every "
                               "connected slave)."));
    connect(m_expandBtn, &QPushButton::toggled, this, [this](bool on){
        m_expandBtn->setText(on ? tr("Collapse ▲") : tr("Expand grid ⤵"));
        emit expandToggled(on);
    });
    row->addWidget(m_expandBtn);
}

void SlavePickerBar::setExpandButtonVisible(bool on)
{
    if (m_expandBtn){ m_expandBtn->setVisible(on); }
}

void SlavePickerBar::bindModel(QAbstractItemModel* model,
                               QItemSelectionModel* tableSelection)
{
    m_model         = model;
    m_tableSelModel = tableSelection;

    if (m_model){
        m_combo->setModel(m_model);
        m_combo->setModelColumn(SlaveTableModel::ColName);
        connect(m_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &SlavePickerBar::onComboChanged);
    }
    if (m_tableSelModel){
        connect(m_tableSelModel, &QItemSelectionModel::selectionChanged,
                this, &SlavePickerBar::onTableSelectionChanged);
    }
}

void SlavePickerBar::onComboChanged(int row)
{
    if (m_syncing){ return; }
    if (row < 0 || !m_model){
        m_currentRow = -1;
        m_currentIdx = -1;
        m_lastStateCode = -1;     /* force restyle on next snapshot */
        return;
    }
    m_currentRow = row;
    m_lastStateCode = -1;          /* new slave → force restyle      */
    /* Cache the slave's logical idx so onSnapshots can filter on it
     * without going back through the model. */
    if (m_model->rowCount() > row){
        m_currentIdx = m_model->index(row, SlaveTableModel::ColIdx)
                              .data().toInt();
    }
    emit slaveSelected(row);
}

void SlavePickerBar::onTableSelectionChanged()
{
    if (!m_tableSelModel){ return; }
    const auto rows = m_tableSelModel->selectedRows();
    if (rows.isEmpty()){ return; }
    const int row = rows.first().row();
    if (row == m_currentRow){ return; }
    m_syncing = true;
    m_combo->setCurrentIndex(row);
    m_currentRow = row;
    m_currentIdx = m_model
        ? m_model->index(row, SlaveTableModel::ColIdx).data().toInt()
        : -1;
    m_syncing = false;
}

void SlavePickerBar::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_currentIdx < 0){ return; }
    for (const auto& s : snaps){
        if (s.idx != m_currentIdx){ continue; }

        /* State badge with colour coding. Falls through to a neutral
         * grey for any state the decoder doesn't recognise. */
        const int code = stateCode(s.statusword);
        const QString name = s.pdoFresh ? stateName(s.statusword)
                                        : QStringLiteral("(no PDO)");
        m_stateLbl->setText(QStringLiteral("● %1").arg(name));
        /* setStyleSheet only on state-code change — it reparses CSS,
         * re-polishes the widget, and schedules a repaint. Calling it
         * 20+ times per second is what made the GUI lag accumulate
         * even after the chart and button-gating fixes. */
        if (code != m_lastStateCode){
            m_lastStateCode = code;
            QString colour = QStringLiteral("#222222");
            switch (code){
            case 4:  colour = QStringLiteral("#1a6e1a"); break;   /* OP_ENABLED – green */
            case 5:  colour = QStringLiteral("#8a5a00"); break;   /* QUICK_STOP – amber */
            case 6:
            case 7:  colour = QStringLiteral("#8a1a1a"); break;   /* FAULT – red        */
            default: break;
            }
            m_stateLbl->setStyleSheet(QStringLiteral(
                "QLabel { color: white; background-color: %1; "
                "         padding: 2px 8px; border-radius: 3px; }").arg(colour));
        }

        m_posLbl->setText(QStringLiteral("q %1%2")
            .arg(s.position >= 0 ? QStringLiteral("+") : QString())
            .arg(double(s.position), 0, 'f', 4));
        m_velLbl->setText(QStringLiteral("ω %1%2")
            .arg(s.velocity >= 0 ? QStringLiteral("+") : QString())
            .arg(double(s.velocity), 0, 'f', 3));
        m_trqLbl->setText(QStringLiteral("τ %1%2")
            .arg(s.torque >= 0 ? QStringLiteral("+") : QString())
            .arg(double(s.torque), 0, 'f', 3));
        m_errLbl->setText(QStringLiteral("err 0x%1")
            .arg(s.errorCode, 4, 16, QChar('0')));
        return;
    }
}

QString SlavePickerBar::stateName(uint16_t sw)
{
    /* CiA-402 statusword decoder. Matches JointControlPanel::stateName
     * verbatim — kept here as a local copy rather than #including the
     * panel header to avoid coupling the bar to the panel. */
    if      ((sw & 0x4F) == 0x00) return QStringLiteral("NOT_READY");
    else if ((sw & 0x4F) == 0x40) return QStringLiteral("SWITCH_ON_DISABLED");
    else if ((sw & 0x6F) == 0x21) return QStringLiteral("READY_TO_SWITCH_ON");
    else if ((sw & 0x6F) == 0x23) return QStringLiteral("SWITCHED_ON");
    else if ((sw & 0x6F) == 0x27) return QStringLiteral("OPERATION_ENABLED");
    else if ((sw & 0x6F) == 0x07) return QStringLiteral("QUICK_STOP_ACTIVE");
    else if ((sw & 0x4F) == 0x0F) return QStringLiteral("FAULT_REACTION_ACTIVE");
    else if ((sw & 0x4F) == 0x08) return QStringLiteral("FAULT");
    else                          return QStringLiteral("UNKNOWN");
}

int SlavePickerBar::stateCode(uint16_t sw)
{
    if      ((sw & 0x4F) == 0x00) return 0;
    else if ((sw & 0x4F) == 0x40) return 1;
    else if ((sw & 0x6F) == 0x21) return 2;
    else if ((sw & 0x6F) == 0x23) return 3;
    else if ((sw & 0x6F) == 0x27) return 4;
    else if ((sw & 0x6F) == 0x07) return 5;
    else if ((sw & 0x4F) == 0x0F) return 6;
    else if ((sw & 0x4F) == 0x08) return 7;
    else                          return 99;
}

}  // namespace vrmc
