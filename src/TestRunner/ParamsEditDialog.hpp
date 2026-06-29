/**
 * @file    ParamsEditDialog.hpp
 * @brief   Modal editor for one test's params block — opens from the
 *          Edit Params… button in @ref TestDescriptionDialog.
 *
 *  Type-aware widgets: each row picks QSpinBox / QDoubleSpinBox /
 *  QCheckBox / QLineEdit based on the existing JSON value's native
 *  type. Result on OK is a fresh QJsonObject the caller persists.
 */
#pragma once

#include <QDialog>
#include <QJsonObject>

class QFormLayout;


class ParamsEditDialog : public QDialog
{
    Q_OBJECT
public:
    /**
     * @param test_id        for the window title — operator sees which test
     *                       they're editing
     * @param params         current values; types here drive widget choice
     * @param param_meanings shown as the widget tooltip + below-label hint
     */
    explicit ParamsEditDialog(const QString& test_id,
                              const QJsonObject& params,
                              const QHash<QString, QString>& param_meanings,
                              QWidget* parent = nullptr);

    /** @brief Read the edited values back. Same keys as the input
     *  params; values reflect what the operator typed. */
    QJsonObject editedParams() const;

private:
    QString          test_id_;
    QJsonObject      params_;     /**< the original — drives type per row */
    QFormLayout*     form_ = nullptr;
    /* Editor widget per key. void* avoids dragging every Qt input
     * widget into this header just to declare a map. Real type is
     * resolved by switching on params_[key].type() inside the impl. */
    QHash<QString, QWidget*> editors_;
};
