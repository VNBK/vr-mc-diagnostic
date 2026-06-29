/*
 * ParamsEditDialog.cpp — see header for purpose.
 */
#include "ParamsEditDialog.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>


ParamsEditDialog::ParamsEditDialog(const QString& test_id,
                                    const QJsonObject& params,
                                    const QHash<QString, QString>& param_meanings,
                                    QWidget* parent)
    : QDialog(parent), test_id_(test_id), params_(params)
{
    setWindowTitle(tr("Edit params — %1").arg(test_id_));
    setModal(true);
    resize(520, 0);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(12, 12, 12, 12);

    auto* header = new QLabel(
        tr("<b>%1</b> — edit JSON-defined parameters. OK persists "
           "back to the loaded tests.json.").arg(test_id_), this);
    header->setWordWrap(true);
    outer->addWidget(header);

    form_ = new QFormLayout();
    form_->setLabelAlignment(Qt::AlignRight);
    outer->addLayout(form_);

    /* Build one row per param, picking widget by JSON type. Integers
     * use QSpinBox; doubles use QDoubleSpinBox; booleans QCheckBox;
     * everything else (string / object / array) falls back to a free-
     * text QLineEdit that round-trips through QJsonValue::fromVariant. */
    for (auto it = params_.constBegin(); it != params_.constEnd(); ++it){
        const QString key = it.key();
        const QJsonValue v = it.value();
        QWidget* editor = nullptr;
        switch (v.type()){
            case QJsonValue::Bool: {
                auto* cb = new QCheckBox(this);
                cb->setChecked(v.toBool());
                editor = cb;
                break;
            }
            case QJsonValue::Double: {
                const double d = v.toDouble();
                /* Detect integer-ish JSON values so the operator gets
                 * a clean spinner instead of "5.000000". JSON doesn't
                 * distinguish int from double; pick by whether the
                 * source string had a decimal point. */
                const bool is_integer = (std::floor(d) == d)
                                       && std::abs(d) < 1e9;
                if (is_integer){
                    auto* sb = new QSpinBox(this);
                    sb->setRange(-1000000000, 1000000000);
                    sb->setValue(int(d));
                    editor = sb;
                } else {
                    auto* sb = new QDoubleSpinBox(this);
                    sb->setRange(-1e9, 1e9);
                    sb->setDecimals(6);
                    sb->setSingleStep(0.01);
                    sb->setValue(d);
                    editor = sb;
                }
                break;
            }
            case QJsonValue::String: {
                auto* le = new QLineEdit(v.toString(), this);
                editor = le;
                break;
            }
            default: {
                /* Object / array / null — JSON-stringified into a
                 * line edit. Operator gets to keep editing them in
                 * raw JSON without us crashing on type mismatch. */
                auto* le = new QLineEdit(v.toVariant().toString(), this);
                editor = le;
                break;
            }
        }
        const QString meaning = param_meanings.value(
            key, tr("(no meaning specified)"));
        editor->setToolTip(meaning);
        /* Two-line label: key on top, meaning below in muted grey.
         * Saves a third column and matches the table layout. */
        auto* lbl = new QLabel(
            QStringLiteral("<b>%1</b><br><span style='color:#666;'>%2</span>")
                .arg(key, meaning.toHtmlEscaped()), this);
        lbl->setWordWrap(true);
        form_->addRow(lbl, editor);
        editors_.insert(key, editor);
    }

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);
}


QJsonObject ParamsEditDialog::editedParams() const
{
    QJsonObject out;
    for (auto it = params_.constBegin(); it != params_.constEnd(); ++it){
        const QString key = it.key();
        QWidget* w = editors_.value(key);
        if (!w){ out.insert(key, it.value()); continue; }
        if (auto* cb = qobject_cast<QCheckBox*>(w)){
            out.insert(key, cb->isChecked());
        } else if (auto* sb = qobject_cast<QSpinBox*>(w)){
            out.insert(key, sb->value());
        } else if (auto* sb = qobject_cast<QDoubleSpinBox*>(w)){
            out.insert(key, sb->value());
        } else if (auto* le = qobject_cast<QLineEdit*>(w)){
            /* Try to round-trip the original JSON value's type — if
             * the user kept it numeric we'd rather store a number
             * than a string. Falls back to string on parse failure. */
            const QString text = le->text();
            const QJsonValue::Type t = it.value().type();
            if (t == QJsonValue::Double){
                bool ok = false;
                const double d = text.toDouble(&ok);
                if (ok){ out.insert(key, d); continue; }
            }
            out.insert(key, text);
        } else {
            out.insert(key, it.value());     /* unknown widget — preserve */
        }
    }
    return out;
}
