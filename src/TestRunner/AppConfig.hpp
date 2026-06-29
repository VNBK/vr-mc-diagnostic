/**
 * @file    AppConfig.hpp
 * @brief   Minimal QSettings-backed key/value store, API-compatible
 *          with the vr_hand_diagnostic AppConfig the cloned TestRunner
 *          calls. Lives inside the TestRunner folder so we don't have
 *          to fork another diagnostic-wide config singleton.
 *
 * Only the two accessors the cloned code uses are surfaced:
 *   - @c value(key, default)
 *   - @c setValue(key, value)
 *
 * Storage backend is QSettings (organisation = "vrmc", application =
 * "vr_mc_diagnostic"), so the persisted file is a plain INI under
 * ~/.config/vrmc/vr_mc_diagnostic.conf. Drop-in replacement: nothing in
 * the cloned files needs to change except the include path.
 */

#pragma once

#include <QSettings>
#include <QString>
#include <QVariant>


class AppConfig
{
public:
    static AppConfig& instance()
    {
        static AppConfig s;
        return s;
    }

    QVariant value(const QString& key,
                    const QVariant& def = {}) const
    {
        return settings_.value(key, def);
    }

    void setValue(const QString& key, const QVariant& val)
    {
        settings_.setValue(key, val);
    }

private:
    AppConfig()
        : settings_(QSettings::IniFormat,
                     QSettings::UserScope,
                     QStringLiteral("vrmc"),
                     QStringLiteral("vr_mc_diagnostic"))
    {}
    AppConfig(const AppConfig&)            = delete;
    AppConfig& operator=(const AppConfig&) = delete;

    mutable QSettings settings_;
};
