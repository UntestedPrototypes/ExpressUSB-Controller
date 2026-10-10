#pragma once
// Glue between the controller input, the control model, the ExpressUSB link and the QML dashboard.
#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVariantList>
#include <memory>

#include <mavlink_controller/mavlink_controller.h>

#include "InputReader.h"
#include "core/ChannelMapper.h"
#include "core/ControlModel.h"

class Backend : public QObject {
    Q_OBJECT
    // controller
    Q_PROPERTY(bool controllerConnected READ controllerConnected NOTIFY tick)
    Q_PROPERTY(QString controllerName READ controllerName NOTIFY tick)
    Q_PROPERTY(int touchpads READ touchpads NOTIFY tick)
    Q_PROPERTY(QVariantMap input READ input NOTIFY tick)
    // control state
    Q_PROPERTY(bool armed READ armed NOTIFY tick)
    Q_PROPERTY(bool estop READ estop NOTIFY tick)
    Q_PROPERTY(double armProgress READ armProgress NOTIFY tick)
    Q_PROPERTY(double resetProgress READ resetProgress NOTIFY tick)
    Q_PROPERTY(QVariantMap control READ control NOTIFY tick)
    Q_PROPERTY(QVariantList channels READ channels NOTIFY tick)
    // link
    Q_PROPERTY(QStringList ports READ ports NOTIFY portsChanged)
    Q_PROPERTY(QString linkState READ linkState NOTIFY tick)
    Q_PROPERTY(QVariantMap telemetry READ telemetry NOTIFY tick)
    Q_PROPERTY(int page READ page WRITE setPage NOTIFY pageChanged)

public:
    explicit Backend(QObject* parent = nullptr);
    ~Backend() override;

    bool controllerConnected() const { return m_in.connected; }
    QString controllerName() const { return m_reader.controllerName(); }
    int touchpads() const { return m_reader.touchpadCount(); }
    QVariantMap input() const;
    bool armed() const { return m_out.armed; }
    bool estop() const { return m_out.estop; }
    double armProgress() const { return m_out.armProgress; }
    double resetProgress() const { return m_out.resetProgress; }
    QVariantMap control() const;
    QVariantList channels() const;
    QStringList ports() const { return m_ports; }
    QString linkState() const;
    QVariantMap telemetry() const;
    int page() const { return m_page; }
    void setPage(int p);

public slots:
    void refreshPorts();
    void connectLink(const QString& port, const QString& mode);   // mode "crsf" / "mavlink"
    void disconnectLink();
    void emergencyStop();      // touch screen button
    void disarm();
    void appInactive();        // focus lost: disarm

signals:
    void tick();
    void portsChanged();
    void pageChanged();

private:
    void step();

    InputReader m_reader;
    deck::ControlModel m_model;
    deck::ChannelMap m_map;
    deck::InputState m_in;
    deck::ControlOutput m_out;
    deck::Channels m_ch;
    mavlink_ctrl::MavlinkController m_link;
    QTimer m_timer;
    QStringList m_ports;
    qint64 m_lastNs = 0;
    int m_page = 0;
    int m_tickCount = 0;
    mavlink_ctrl::TelemetryData m_tel;
};
