/**
 * @file   MotorProfileView.hpp
 * @brief  Read-only table showing the loaded @ref MotorParams fields.
 *
 * Lives as a tab in the main window so the user can see the active
 * motor profile at a glance. Refreshes whenever the cached params are
 * updated (Open Profile, Edit Motor Profile). The "Edit…" button at the
 * bottom emits @ref editRequested so MainWindow can pop the modal
 * editor without this widget knowing about it.
 */

#pragma once

#include "MotorProfile.hpp"
#include <QWidget>

class QTableWidget;
class QPushButton;

namespace vrmc {

class MotorProfileView : public QWidget
{
    Q_OBJECT
public:
    explicit MotorProfileView(QWidget* parent = nullptr);

public slots:
    void setParams(const vrmc::MotorParams& mp);

signals:
    void editRequested();

private:
    void   setRow(int row, const QString& name,
                  const QString& value, const QString& unit = {});

    QTableWidget* m_table = nullptr;
    QPushButton*  m_edit  = nullptr;
};

}  // namespace vrmc
