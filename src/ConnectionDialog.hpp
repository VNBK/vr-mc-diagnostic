/**
 * @file   ConnectionDialog.hpp
 * @brief  Modal dialog collecting transport choice (UDP / ZLG) plus
 *         backend-specific endpoint params and slave-id range.
 */

#pragma once

#include "backends/CanBackend.hpp"
#include <QDialog>

class QComboBox;
class QGroupBox;
class QLineEdit;
class QSpinBox;

namespace vrmc {

class ConnectionDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ConnectionDialog(QWidget* parent = nullptr);

    CanConfig config() const;

    /** Pre-fill for attaching to a board in its bootloader: node id + a
     *  single-slave count. The operator still picks the transport (UDP sim
     *  vs ZLG hardware). MainWindow forces allow_offline on the result. */
    void presetForBootloader(int nodeId);

private slots:
    void onTransportChanged();

private:
    QComboBox* m_transport = nullptr;

    /* UDP-specific. */
    QGroupBox* m_udpBox    = nullptr;
    QLineEdit* m_group     = nullptr;
    QSpinBox*  m_port      = nullptr;

    /* ZLG-specific. */
    QGroupBox* m_zlgBox    = nullptr;
    QSpinBox*  m_zlgChan   = nullptr;
    QSpinBox*  m_zlgArb    = nullptr;
    QSpinBox*  m_zlgFd     = nullptr;

    /* Common. */
    QSpinBox*  m_firstId   = nullptr;
    QSpinBox*  m_count     = nullptr;
    QSpinBox*  m_timeout   = nullptr;
};

}  // namespace vrmc
