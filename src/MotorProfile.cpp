#include "MotorProfile.hpp"

#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

namespace vrmc {
namespace profile {

static constexpr int kSchemaVersion = 2;

QString motorTypeToString(MotorType t)
{
    switch (t){
    case MotorType::Bldc: return QStringLiteral("BLDC");
    case MotorType::Pmsm: return QStringLiteral("PMSM");
    }
    return QStringLiteral("PMSM");
}

MotorType motorTypeFromString(const QString& s)
{
    const QString u = s.toUpper();
    if (u == QStringLiteral("BLDC")){ return MotorType::Bldc; }
    return MotorType::Pmsm;
}

/* ------------------------------------------------------------------ *
 *  Connection block (kept so a profile can also restore the session) *
 * ------------------------------------------------------------------ */

static QJsonObject encodeConnection(const CanConfig& c)
{
    QJsonObject conn;
    conn[QStringLiteral("kind")] = (c.kind == CanKind::Zlg)
                                       ? QStringLiteral("zlg")
                                       : QStringLiteral("udp");

    QJsonObject udp;
    udp[QStringLiteral("group")] = c.group;
    udp[QStringLiteral("port")]  = static_cast<int>(c.port);
    conn[QStringLiteral("udp")]  = udp;

    QJsonObject zlg;
    zlg[QStringLiteral("channel")] = static_cast<int>(c.zlgChannel);
    zlg[QStringLiteral("arb_bps")] = static_cast<qint64>(c.zlgBitrate);
    zlg[QStringLiteral("fd_bps")]  = static_cast<qint64>(c.zlgFdBitrate);
    conn[QStringLiteral("zlg")]    = zlg;

    conn[QStringLiteral("first_id")]       = static_cast<int>(c.first_id);
    conn[QStringLiteral("count")]          = static_cast<int>(c.count);
    conn[QStringLiteral("sdo_timeout_ms")] = static_cast<int>(c.sdo_timeout_ms);
    return conn;
}

static void decodeConnection(const QJsonObject& conn, CanConfig& c)
{
    const auto kind = conn.value(QStringLiteral("kind"))
                          .toString(QStringLiteral("udp")).toLower();
    c.kind = (kind == QStringLiteral("zlg")) ? CanKind::Zlg : CanKind::Udp;

    const auto udp = conn.value(QStringLiteral("udp")).toObject();
    c.group = udp.value(QStringLiteral("group"))
                  .toString(QStringLiteral("239.192.0.42"));
    c.port  = static_cast<uint16_t>(udp.value(QStringLiteral("port")).toInt(23400));

    const auto zlg = conn.value(QStringLiteral("zlg")).toObject();
    c.zlgChannel  = static_cast<uint32_t>(zlg.value(QStringLiteral("channel")).toInt(0));
    c.zlgBitrate  = static_cast<uint32_t>(zlg.value(QStringLiteral("arb_bps")).toInt(1000000));
    c.zlgFdBitrate = static_cast<uint32_t>(zlg.value(QStringLiteral("fd_bps")).toInt(4000000));

    c.first_id       = static_cast<uint8_t>(conn.value(QStringLiteral("first_id")).toInt(5));
    c.count          = static_cast<uint8_t>(conn.value(QStringLiteral("count")).toInt(1));
    c.sdo_timeout_ms = static_cast<uint32_t>(conn.value(QStringLiteral("sdo_timeout_ms")).toInt(100));
}

/* ------------------------------------------------------------------ *
 *  Motor block — name-plate + electrical                             *
 * ------------------------------------------------------------------ */

static QJsonObject encodeMotor(const MotorParams& m)
{
    QJsonObject obj;
    obj[QStringLiteral("type")]           = motorTypeToString(m.type);
    obj[QStringLiteral("pole_pair")]      = static_cast<int>(m.pole_pair);
    obj[QStringLiteral("rs")]             = static_cast<double>(m.rs);
    obj[QStringLiteral("ls_d")]           = static_cast<double>(m.ls_d);
    obj[QStringLiteral("ls_q")]           = static_cast<double>(m.ls_q);
    obj[QStringLiteral("rated_flux")]     = static_cast<double>(m.rated_flux);
    obj[QStringLiteral("inertia")]        = static_cast<double>(m.inertia);
    obj[QStringLiteral("rated_torque")]   = static_cast<double>(m.rated_torque);
    obj[QStringLiteral("rated_speed")]    = m.rated_speed;
    obj[QStringLiteral("rated_vol")]      = m.rated_vol;
    obj[QStringLiteral("rated_cur")]      = m.rated_cur;
    obj[QStringLiteral("enc_increments")] = static_cast<qint64>(m.enc_increments);
    obj[QStringLiteral("enc_motor_revs")] = static_cast<qint64>(m.enc_motor_revs);
    return obj;
}

static void decodeMotor(const QJsonObject& obj, MotorParams& m)
{
    m.type           = motorTypeFromString(
                          obj.value(QStringLiteral("type")).toString(QStringLiteral("PMSM")));
    m.pole_pair      = static_cast<uint32_t>(obj.value(QStringLiteral("pole_pair")).toInt(4));
    m.rs             = static_cast<float>(obj.value(QStringLiteral("rs")).toDouble(0.5));
    m.ls_d           = static_cast<float>(obj.value(QStringLiteral("ls_d")).toDouble(0.0015));
    m.ls_q           = static_cast<float>(obj.value(QStringLiteral("ls_q")).toDouble(0.0015));
    m.rated_flux     = static_cast<float>(obj.value(QStringLiteral("rated_flux")).toDouble(0.05));
    m.inertia        = static_cast<float>(obj.value(QStringLiteral("inertia")).toDouble(5.0e-5));
    m.rated_torque   = static_cast<float>(obj.value(QStringLiteral("rated_torque")).toDouble(0.5));
    m.rated_speed    = obj.value(QStringLiteral("rated_speed"))   .toInt(1000);
    m.rated_vol      = obj.value(QStringLiteral("rated_vol"))     .toInt(24);
    m.rated_cur      = obj.value(QStringLiteral("rated_cur"))     .toInt(2);
    m.enc_increments = static_cast<uint32_t>(obj.value(QStringLiteral("enc_increments")).toInt(16384));
    m.enc_motor_revs = static_cast<uint32_t>(obj.value(QStringLiteral("enc_motor_revs")).toInt(1));
}

/* ------------------------------------------------------------------ *
 *  Public API                                                        *
 * ------------------------------------------------------------------ */

bool save(const QString& path, const MotorProfile& mp, QString* err)
{
    QJsonObject root;
    root[QStringLiteral("version")]    = kSchemaVersion;
    root[QStringLiteral("motor")]      = encodeMotor(mp.motor);
    root[QStringLiteral("connection")] = encodeConnection(mp.connection);

    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (err){ *err = QStringLiteral("cannot open %1 for write").arg(path); }
        return false;
    }
    const auto bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (f.write(bytes) != bytes.size()){
        if (err){ *err = QStringLiteral("short write to %1").arg(path); }
        return false;
    }
    if (!f.commit()){
        if (err){ *err = QStringLiteral("commit failed for %1").arg(path); }
        return false;
    }
    return true;
}

bool load(const QString& path, MotorProfile* mp_out, QString* err)
{
    if (!mp_out){ return false; }
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)){
        if (err){ *err = QStringLiteral("cannot open %1").arg(path); }
        return false;
    }
    QJsonParseError perr{};
    const auto doc = QJsonDocument::fromJson(f.readAll(), &perr);
    if (perr.error != QJsonParseError::NoError){
        if (err){ *err = QStringLiteral("JSON: %1").arg(perr.errorString()); }
        return false;
    }
    const auto root = doc.object();
    mp_out->schemaVersion = root.value(QStringLiteral("version")).toInt(1);
    if (mp_out->schemaVersion > kSchemaVersion){
        if (err){
            *err = QStringLiteral("profile schema v%1 newer than supported v%2")
                       .arg(mp_out->schemaVersion).arg(kSchemaVersion);
        }
        return false;
    }
    decodeMotor     (root.value(QStringLiteral("motor"))     .toObject(), mp_out->motor);
    decodeConnection(root.value(QStringLiteral("connection")).toObject(), mp_out->connection);
    return true;
}

}  // namespace profile
}  // namespace vrmc
