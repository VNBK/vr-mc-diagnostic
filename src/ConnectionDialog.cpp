#include "ConnectionDialog.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>

namespace vrmc {

ConnectionDialog::ConnectionDialog(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Connect — CiA 402"));

    /* --- transport selector --- */
    m_transport = new QComboBox(this);
    /* ZLG USB-CANFD first → default when the dialog opens. UDP
     * loopback (Help → Start demo) is the second choice and stays
     * easy to reach. */
    m_transport->addItem(tr("ZLG USB-CANFD (real hardware)"),
                         int(CanKind::Zlg));
    m_transport->addItem(tr("UDP multicast (loopback simulator)"),
                         int(CanKind::Udp));
    m_transport->addItem(tr("Feetech SMS/STS (RS-485 bus)"),
                         int(CanKind::Feetech));
    m_transport->addItem(tr("ROBOTIS Dynamixel (RS-485 bus)"),
                         int(CanKind::Dynamixel));
    connect(m_transport,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int){ onTransportChanged(); });

    /* --- UDP rows --- */
    m_udpBox = new QGroupBox(tr("UDP transport"), this);
    m_group  = new QLineEdit(QStringLiteral("239.192.0.42"), m_udpBox);
    m_port   = new QSpinBox(m_udpBox);   m_port->setRange(1, 65535);
    m_port->setValue(23400);
    auto* udpForm = new QFormLayout(m_udpBox);
    udpForm->addRow(tr("Multicast group"), m_group);
    udpForm->addRow(tr("UDP port"),         m_port);

    /* --- ZLG rows --- */
    m_zlgBox = new QGroupBox(tr("ZLG USB-CANFD"), this);
    m_zlgChan = new QSpinBox(m_zlgBox); m_zlgChan->setRange(0, 7);
    m_zlgChan->setValue(0);
    m_zlgArb  = new QSpinBox(m_zlgBox); m_zlgArb->setRange(50000, 1000000);
    m_zlgArb->setSuffix(tr(" bps"));    m_zlgArb->setSingleStep(50000);
    m_zlgArb->setValue(500000);
    m_zlgFd   = new QSpinBox(m_zlgBox); m_zlgFd->setRange(0, 8000000);
    m_zlgFd->setSuffix(tr(" bps"));     m_zlgFd->setSingleStep(500000);
    m_zlgFd->setValue(2000000);
    auto* zlgForm = new QFormLayout(m_zlgBox);
    zlgForm->addRow(tr("Channel"),          m_zlgChan);
    zlgForm->addRow(tr("Arb bitrate"),      m_zlgArb);
    zlgForm->addRow(tr("Data bitrate (FD; 0 disables)"), m_zlgFd);

    /* --- UART rows (Feetech + Dynamixel share this group) --- */
    m_uartBox  = new QGroupBox(tr("UART (Feetech / Dynamixel)"), this);
    m_uartDev  = new QLineEdit(QStringLiteral("/dev/ttyUSB0"), m_uartBox);
    m_uartBaud = new QSpinBox(m_uartBox);
    m_uartBaud->setRange(9600, 4'500'000);
    m_uartBaud->setSingleStep(100'000);
    m_uartBaud->setValue(1'000'000);
    m_uartBaud->setSuffix(tr(" bps"));
    m_dxlProto = new QSpinBox(m_uartBox);
    m_dxlProto->setRange(1, 2);
    m_dxlProto->setValue(2);
    auto* uartForm = new QFormLayout(m_uartBox);
    uartForm->addRow(tr("Serial device"),    m_uartDev);
    uartForm->addRow(tr("Baudrate"),         m_uartBaud);
    uartForm->addRow(tr("Dynamixel proto"),  m_dxlProto);

    /* --- common rows --- */
    m_firstId = new QSpinBox(this);   m_firstId->setRange(1, 127);
    m_firstId->setValue(5);
    m_count   = new QSpinBox(this);   m_count->setRange(1, 32);
    m_count->setValue(1);
    m_timeout = new QSpinBox(this);   m_timeout->setRange(1, 5000);
    m_timeout->setValue(100);
    m_timeout->setSuffix(QStringLiteral(" ms"));

    auto* commonForm = new QFormLayout;
    commonForm->addRow(tr("First node ID"),  m_firstId);
    commonForm->addRow(tr("Slave count"),    m_count);
    commonForm->addRow(tr("SDO timeout"),    m_timeout);

    /* --- root layout --- */
    auto* topForm = new QFormLayout;
    topForm->addRow(tr("Transport"), m_transport);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* root = new QVBoxLayout(this);
    root->addLayout(topForm);
    root->addWidget(m_udpBox);
    root->addWidget(m_zlgBox);
    root->addWidget(m_uartBox);
    root->addLayout(commonForm);
    root->addWidget(buttons);

    onTransportChanged();
}

void ConnectionDialog::onTransportChanged()
{
    const auto kind = static_cast<CanKind>(m_transport->currentData().toInt());
    m_udpBox->setVisible(kind == CanKind::Udp);
    m_zlgBox->setVisible(kind == CanKind::Zlg);
    const bool uartKind = (kind == CanKind::Feetech ||
                           kind == CanKind::Dynamixel);
    m_uartBox->setVisible(uartKind);
    /* Proto version only relevant for Dynamixel. */
    if (m_dxlProto){
        m_dxlProto->setEnabled(kind == CanKind::Dynamixel);
    }
    /* Feetech default baud is typically 1 Mbps; Dynamixel commonly runs
     * at 57 600 or 1 Mbps. Nudge the field to a sensible default on
     * kind change if it's still sitting at the opposite's preset. */
    if (kind == CanKind::Feetech && m_uartBaud->value() < 500'000){
        m_uartBaud->setValue(1'000'000);
    }
    adjustSize();
}

CanConfig ConnectionDialog::config() const
{
    CanConfig c;
    c.kind           = static_cast<CanKind>(m_transport->currentData().toInt());
    c.group          = m_group->text();
    c.port           = static_cast<uint16_t>(m_port->value());
    /* zlgLibPath stays at its default (libcontrolcanfd.so + RPATH); no
     * UI control for it any more. */
    c.zlgChannel     = static_cast<uint32_t>(m_zlgChan->value());
    c.zlgBitrate     = static_cast<uint32_t>(m_zlgArb->value());
    c.zlgFdBitrate   = static_cast<uint32_t>(m_zlgFd->value());
    c.uartDevice     = m_uartDev->text();
    c.uartBaud       = static_cast<uint32_t>(m_uartBaud->value());
    c.dxlProtocolVer = static_cast<uint8_t>(m_dxlProto->value());
    c.first_id       = static_cast<uint8_t>(m_firstId->value());
    c.count           = static_cast<uint8_t>(m_count->value());
    c.sdo_timeout_ms = static_cast<uint32_t>(m_timeout->value());
    return c;
}

}  // namespace vrmc
