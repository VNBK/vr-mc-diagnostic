/**
 * @file    DatabaseEditDialog.hpp
 * @brief   Simple modal that edits the metadata of one TestDatabase —
 *          id, name, description. Used by both the Add Database flow
 *          (the runner pre-fills no fields) and the Edit Database flow
 *          (caller loads the current values via @ref loadFromValues).
 *
 *  Database-level edits are deliberately metadata-only — moving a
 *  test out of one DB into another isn't supported here; for that,
 *  edit the test (which locks parent-DB) by hand or use Export /
 *  Import with a hand-tuned tests.json.
 */
#pragma once

#include <QDialog>

class QLineEdit;
class QPlainTextEdit;


class DatabaseEditDialog : public QDialog
{
    Q_OBJECT
public:
    enum class Mode { Add, Edit };

    explicit DatabaseEditDialog(Mode mode, QWidget* parent = nullptr);

    /** @brief Pre-fill all three fields for Edit mode. ID stays
     *  editable in case the operator wants to rename — runner is
     *  responsible for rewriting every test's db_id in the file. */
    void loadFromValues(const QString& id,
                         const QString& name,
                         const QString& description);

    QString id()          const;
    QString name()        const;
    QString description() const;

private:
    Mode             mode_;
    QLineEdit*       id_edit_          = nullptr;
    QLineEdit*       name_edit_        = nullptr;
    QPlainTextEdit*  description_edit_ = nullptr;
};
