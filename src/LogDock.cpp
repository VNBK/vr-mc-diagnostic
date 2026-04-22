#include "LogDock.hpp"

#include <QDateTime>
#include <QPlainTextEdit>

namespace vrmc {

LogDock::LogDock(QWidget* parent) : QDockWidget(tr("Log"), parent)
{
    m_text = new QPlainTextEdit(this);
    m_text->setReadOnly(true);
    m_text->setMaximumBlockCount(1000);
    m_text->setStyleSheet(
        QStringLiteral("QPlainTextEdit{background:#111;color:#ddd;"
                       "font-family:monospace;}"));
    setWidget(m_text);
}

void LogDock::appendInfo(const QString& msg)
{
    append(QStringLiteral("INFO "), msg, QStringLiteral("#9ec6ff"));
}

void LogDock::appendError(const QString& msg)
{
    append(QStringLiteral("ERR  "), msg, QStringLiteral("#ff7070"));
}

void LogDock::append(const QString& level, const QString& msg, const QString& colour)
{
    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    m_text->appendHtml(
        QStringLiteral("<span style='color:#888'>%1</span> "
                       "<span style='color:%2'>%3</span> "
                       "<span style='color:#ddd'>%4</span>")
            .arg(ts, colour, level, msg.toHtmlEscaped()));
}

}  // namespace vrmc
