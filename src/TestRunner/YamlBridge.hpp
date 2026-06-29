/**
 * @file    YamlBridge.hpp
 * @brief   Thin bridge between YAML on disk and the QJson* tree used by
 *          TestRunnerWindow in memory.
 *
 * Rationale: the TestRunner historically persisted tests as JSON and
 * manipulates them as @c QJsonDocument / @c QJsonObject in dozens of
 * places. We're switching the on-disk format to YAML for hand-edit
 * friendliness (comments, fewer braces, easier diff), but rather than
 * rewrite every in-memory mutation path we keep @c QJsonDocument as
 * the working representation and translate at the file boundary only.
 *
 * Round-trip semantics:
 *   - YAML scalars: typed via syntax — bare numbers parse as ints/
 *     doubles, "true"/"false"/"yes"/"no" as bools, "~"/"null"/empty
 *     as null, everything else as string.
 *   - YAML map  → QJsonObject
 *   - YAML seq  → QJsonArray
 *   - JSON int  emits as a bare YAML int (no quoting)
 *   - JSON bool emits as `true` / `false`
 *   - JSON null emits as `~`
 *   - JSON array/object emit as block-style YAML for diff-friendliness
 *
 * YAML comments are NOT preserved across a load → mutate → save cycle
 * (the in-memory tree has no comment slots). That matches the JSON
 * lineage where comments never existed.
 */

#pragma once

#include <QByteArray>
#include <QJsonDocument>
#include <QString>


namespace yaml_bridge {

/** @brief Parse @p bytes as YAML and return it as a QJsonDocument.
 *  Returns a null document and writes a human-readable error into
 *  @p out_error on failure. */
QJsonDocument loadYamlAsJson(const QByteArray& bytes, QString* out_error);

/** @brief Emit @p doc as block-style YAML. Sequences and maps each
 *  take a new line so a diff stays one-edit-per-line. */
QByteArray dumpJsonAsYaml(const QJsonDocument& doc);

}  // namespace yaml_bridge
