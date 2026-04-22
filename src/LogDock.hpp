/**
 * @file   LogDock.hpp
 * @brief  Dockable log tail with severity-colouring.
 */

#pragma once

#include <QDockWidget>

class QPlainTextEdit;

namespace vrmc {

class LogDock : public QDockWidget
{
    Q_OBJECT
public:
    explicit LogDock(QWidget* parent = nullptr);

public slots:
    void appendInfo (const QString& msg);
    void appendError(const QString& msg);

private:
    void append(const QString& level, const QString& msg, const QString& colour);

    QPlainTextEdit* m_text = nullptr;
};

}  // namespace vrmc
