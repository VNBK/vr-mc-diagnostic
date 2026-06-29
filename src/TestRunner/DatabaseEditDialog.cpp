/*
 * DatabaseEditDialog.cpp — see header.
 */
#include "DatabaseEditDialog.hpp"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QVBoxLayout>


DatabaseEditDialog::DatabaseEditDialog(Mode mode, QWidget* parent)
    : QDialog(parent), mode_(mode)
{
    setWindowTitle(mode_ == Mode::Add ? tr("Add database")
                                       : tr("Edit database"));
    setModal(true);
    resize(520, 0);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(12, 12, 12, 12);

    auto* hint = new QLabel(
        mode_ == Mode::Add
            ? tr("Create a new TestDatabase block. Tests are added "
                 "afterwards via the Test menu.")
            : tr("Edit the database header. Renaming the ID rewrites "
                 "every test's parent reference inside this file."),
        this);
    hint->setWordWrap(true);
    outer->addWidget(hint);

    auto* form = new QFormLayout();
    id_edit_ = new QLineEdit(this);
    id_edit_->setPlaceholderText(tr("e.g. DB3"));
    form->addRow(tr("ID:"), id_edit_);

    name_edit_ = new QLineEdit(this);
    name_edit_->setPlaceholderText(tr("e.g. Safety"));
    form->addRow(tr("Name:"), name_edit_);

    description_edit_ = new QPlainTextEdit(this);
    description_edit_->setPlaceholderText(
        tr("What this database covers — the operator reads this from "
           "the description popup when picking the DB at run start."));
    description_edit_->setMaximumHeight(120);
    form->addRow(tr("Description:"), description_edit_);
    outer->addLayout(form);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]{
        if (id_edit_->text().trimmed().isEmpty()){
            id_edit_->setFocus(); return;
        }
        if (name_edit_->text().trimmed().isEmpty()){
            name_edit_->setFocus(); return;
        }
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);
}


void DatabaseEditDialog::loadFromValues(const QString& id,
                                          const QString& name,
                                          const QString& description)
{
    id_edit_->setText(id);
    name_edit_->setText(name);
    description_edit_->setPlainText(description);
}


QString DatabaseEditDialog::id()          const { return id_edit_->text().trimmed(); }
QString DatabaseEditDialog::name()        const { return name_edit_->text().trimmed(); }
QString DatabaseEditDialog::description() const { return description_edit_->toPlainText(); }
