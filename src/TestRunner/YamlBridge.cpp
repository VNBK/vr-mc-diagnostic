/*
 * YamlBridge.cpp — see header for design notes.
 */

#include "YamlBridge.hpp"

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace yaml_bridge {

namespace {

/* ---- YAML → QJsonValue ----------------------------------------- */

/* Re-parse a YAML scalar string as bool / int / double / null /
 * string. yaml-cpp gives us the raw text from Scalar(); we mirror
 * the implicit typing yaml-cpp does internally for Node::as<T>(). */
QJsonValue scalarToJson(const std::string& s)
{
    if (s.empty()){ return QJsonValue::Null; }

    /* Reserved null forms (case-insensitive per YAML 1.2). */
    static const std::string nullForms[] = {
        "~", "null", "Null", "NULL"
    };
    for (const auto& n : nullForms){
        if (s == n){ return QJsonValue::Null; }
    }

    /* Bool — accept the YAML 1.1 set so we round-trip hand-typed
     * "yes"/"no" but emit canonical "true"/"false" on dump. */
    static const std::pair<std::string, bool> bools[] = {
        {"true",  true},  {"True",  true},  {"TRUE",  true},
        {"false", false}, {"False", false}, {"FALSE", false},
        {"yes",   true},  {"Yes",   true},  {"YES",   true},
        {"no",    false}, {"No",    false}, {"NO",    false},
    };
    for (const auto& kv : bools){
        if (s == kv.first){ return QJsonValue(kv.second); }
    }

    /* Try int first — strtoll is strict-ish on stray characters. */
    try {
        size_t consumed = 0;
        const long long iv = std::stoll(s, &consumed);
        if (consumed == s.size()){ return QJsonValue((double)iv); }
    } catch (...) { /* fall through to double */ }

    try {
        size_t consumed = 0;
        const double dv = std::stod(s, &consumed);
        if (consumed == s.size()){ return QJsonValue(dv); }
    } catch (...) { /* fall through to string */ }

    return QJsonValue(QString::fromStdString(s));
}

QJsonValue nodeToJson(const YAML::Node& n)
{
    switch (n.Type()){
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
        return QJsonValue::Null;
    case YAML::NodeType::Scalar:
        return scalarToJson(n.Scalar());
    case YAML::NodeType::Sequence: {
        QJsonArray a;
        for (const auto& child : n){
            a.append(nodeToJson(child));
        }
        return a;
    }
    case YAML::NodeType::Map: {
        QJsonObject o;
        for (const auto& kv : n){
            const QString key = QString::fromStdString(kv.first.Scalar());
            o.insert(key, nodeToJson(kv.second));
        }
        return o;
    }
    }
    return QJsonValue::Null;
}


/* ---- QJsonValue → YAML emitter ------------------------------- */

void emitJson(YAML::Emitter& out, const QJsonValue& v);

void emitArray(YAML::Emitter& out, const QJsonArray& a)
{
    out << YAML::BeginSeq;
    for (const QJsonValue& v : a){ emitJson(out, v); }
    out << YAML::EndSeq;
}

void emitObject(YAML::Emitter& out, const QJsonObject& o)
{
    out << YAML::BeginMap;
    for (auto it = o.constBegin(); it != o.constEnd(); ++it){
        out << YAML::Key   << it.key().toStdString();
        out << YAML::Value;
        emitJson(out, it.value());
    }
    out << YAML::EndMap;
}

void emitJson(YAML::Emitter& out, const QJsonValue& v)
{
    switch (v.type()){
    case QJsonValue::Null:
        out << YAML::Null;
        return;
    case QJsonValue::Bool:
        out << v.toBool();
        return;
    case QJsonValue::Double: {
        /* QJsonValue stores all numbers as double; emit as int when
         * the value is an exact integer to keep YAML readable
         * ("num_samples: 12" rather than "num_samples: 12.0"). */
        const double d = v.toDouble();
        if (std::isfinite(d)
         && d == std::trunc(d)
         && d >= -9007199254740992.0
         && d <=  9007199254740992.0){
            out << (long long)d;
        } else {
            out << d;
        }
        return;
    }
    case QJsonValue::String:
        out << v.toString().toStdString();
        return;
    case QJsonValue::Array:
        emitArray(out, v.toArray());
        return;
    case QJsonValue::Object:
        emitObject(out, v.toObject());
        return;
    case QJsonValue::Undefined:
    default:
        out << YAML::Null;
        return;
    }
}

}  // namespace


QJsonDocument loadYamlAsJson(const QByteArray& bytes, QString* out_error)
{
    try {
        const YAML::Node root = YAML::Load(
            std::string(bytes.constData(), (size_t)bytes.size()));
        const QJsonValue v = nodeToJson(root);
        if (v.isObject()){ return QJsonDocument(v.toObject()); }
        if (v.isArray ()){ return QJsonDocument(v.toArray()); }
        if (out_error){
            *out_error = QStringLiteral(
                "YAML root must be a map or sequence");
        }
        return QJsonDocument{};
    } catch (const YAML::Exception& e){
        if (out_error){
            *out_error = QString("YAML parse error at line %1 col %2: %3")
                            .arg(e.mark.line + 1)
                            .arg(e.mark.column + 1)
                            .arg(QString::fromStdString(e.msg));
        }
        return QJsonDocument{};
    } catch (const std::exception& e){
        if (out_error){
            *out_error = QString("YAML parse error: %1").arg(e.what());
        }
        return QJsonDocument{};
    }
}


QByteArray dumpJsonAsYaml(const QJsonDocument& doc)
{
    YAML::Emitter out;
    /* Block style throughout — diffs stay one-edit-per-line. */
    out.SetIndent(2);
    out.SetMapFormat(YAML::Block);
    out.SetSeqFormat(YAML::Block);

    if (doc.isObject()){
        emitObject(out, doc.object());
    } else if (doc.isArray()){
        emitArray(out, doc.array());
    } else {
        out << YAML::Null;
    }
    QByteArray bytes(out.c_str());
    /* yaml-cpp doesn't append a trailing newline; POSIX tools want one. */
    if (!bytes.endsWith('\n')){ bytes.append('\n'); }
    return bytes;
}

}  // namespace yaml_bridge
