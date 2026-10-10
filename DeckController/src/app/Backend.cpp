#include "Backend.h"

#include <QElapsedTimer>
#include <QTime>
#include <QVariantMap>

using mavlink_ctrl::ConnectionState;
using mavlink_ctrl::LinkMode;

namespace {
QElapsedTimer g_clock;
}

Backend::Backend(QObject* parent) : QObject(parent) {
    g_clock.start();
    refreshPorts();
    m_link.onCrsfFrame([this](uint8_t type, const uint8_t* p, size_t n) {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        if (m_frames.size() < 256) m_frames.push_back({type, std::vector<uint8_t>(p, p + n)});
    });
    connect(&m_timer, &QTimer::timeout, this, &Backend::step);
    m_timer.setTimerType(Qt::PreciseTimer);
    m_timer.start(4);   // 250 Hz
}

Backend::~Backend() {
    m_timer.stop();
    m_link.disconnect();
}

void Backend::setPage(int p) {
    p = ((p % 4) + 4) % 4;
    if (p == m_page) return;
    m_page = p;
    emit pageChanged();
}

void Backend::refreshPorts() {
    m_ports.clear();
    for (const auto& p : mavlink_ctrl::MavlinkController::listSerialPorts()) m_ports << QString::fromStdString(p);
    emit portsChanged();
}

void Backend::connectLink(const QString& port, const QString& mode) {
    m_link.disconnect();
    LinkMode lm = LinkMode::Crsf;
    mavlink_ctrl::MavlinkController::parseLinkMode(mode.toStdString(), lm);
    m_link.setLinkMode(lm);
    m_parser.reset();
    m_link.connect(port.toStdString());
    addLog(QStringLiteral("Connecting to %1 (%2)").arg(port, mode));
}

void Backend::disconnectLink() { m_link.disconnect(); }

void Backend::emergencyStop() { m_model.emergencyStop(); }
void Backend::disarm() { m_model.disarm(); }
void Backend::appInactive() { m_model.disarm(); }

void Backend::step() {
    const qint64 now = g_clock.nsecsElapsed();
    float dt = m_lastNs ? float((now - m_lastNs) * 1e-9) : 0.004f;
    m_lastNs = now;
    if (dt > 0.1f) dt = 0.1f;     // a stall must not count as a long hold

    {   // robot telemetry
        std::vector<Frame> frames;
        {
            std::lock_guard<std::mutex> lock(m_frameMutex);
            frames.swap(m_frames);
        }
        const uint32_t ms = uint32_t(now / 1000000);
        for (const Frame& f : frames) {
            const char oldPosture = m_parser.state().posture;
            const bool wasKnown = m_parser.state().haveStatus;
            if (m_parser.onFrame(f.type, f.payload.data(), f.payload.size(), ms)) {
                const deck::RobotState& r = m_parser.state();
                QString line = QStringLiteral("%1").arg(deck::postureName(r.posture));
                if (wasKnown && oldPosture != r.posture) line = QStringLiteral("%1 -> %2").arg(deck::postureName(oldPosture), deck::postureName(r.posture));
                if (r.fault) line += QStringLiteral("  FAULT %1").arg(deck::faultName(r.fault));
                else if (r.stopReason) line += QStringLiteral("  stopping: %1").arg(deck::faultName(r.stopReason));
                if (!r.text.empty()) line += QStringLiteral("  (%1)").arg(QString::fromStdString(r.text));
                addLog(line);
            }
        }
    }

    m_in = m_reader.poll();
    m_out = m_model.update(m_in, dt);
    if (m_out.nextPage) setPage(m_page + 1);
    m_ch = deck::mapChannels(m_out, m_map);

    // The channels go out continuously; disarmed and E-stop are values like the others
    for (int i = 0; i < deck::kChannels; ++i) m_link.setChannel(uint8_t(i), m_ch.us[size_t(i)]);

    if (++m_tickCount % 8 == 0) {          // 30 Hz for the screen
        m_tel = m_link.getTelemetry();
        emit tick();
    }
}

QVariantMap Backend::input() const {
    QVariantMap m;
    m["leftX"] = m_in.leftX;    m["leftY"] = m_in.leftY;
    m["rightX"] = m_in.rightX;  m["rightY"] = m_in.rightY;
    m["r2"] = m_in.r2;
    m["lpDown"] = m_in.leftPad.down;   m["lpX"] = m_in.leftPad.x;   m["lpY"] = m_in.leftPad.y;
    m["rpDown"] = m_in.rightPad.down;  m["rpX"] = m_in.rightPad.x;  m["rpY"] = m_in.rightPad.y;
    return m;
}

QVariantMap Backend::control() const {
    QVariantMap m;
    m["strafe"] = m_out.strafe;  m["forward"] = m_out.forward;
    m["height"] = m_out.height;  m["turn"] = m_out.turn;
    m["roll"] = m_out.roll;      m["pitch"] = m_out.pitch;   m["stance"] = m_out.stance;
    m["ripple"] = m_out.gaitRipple;
    m["solver"] = m_out.solver;
    m["stepHeight"] = m_out.stepHeight;
    m["tiltRecorded"] = m_out.tiltRecorded;
    m["stanceRecorded"] = m_out.stanceRecorded;
    return m;
}

QVariantList Backend::channels() const {
    QVariantList l;
    for (auto v : m_ch.us) l << int(v);
    return l;
}

QString Backend::linkState() const {
    switch (m_link.getState()) {
    case ConnectionState::Disconnected: return QStringLiteral("Disconnected");
    case ConnectionState::Connected: return QStringLiteral("Port open");
    case ConnectionState::LinkUp: return QStringLiteral("Link up");
    }
    return {};
}

void Backend::addLog(const QString& text) {
    m_log.prepend(QTime::currentTime().toString("HH:mm:ss") + "  " + text);
    while (m_log.size() > 200) m_log.removeLast();
    emit logChanged();
}

QVariantMap Backend::robot() const {
    const deck::RobotState& r = m_parser.state();
    QVariantMap m;
    m["known"] = r.haveStatus;
    m["posture"] = QString::fromUtf8(deck::postureName(r.posture));
    m["postureCode"] = QString(QChar(r.posture));
    m["state"] = QString::fromUtf8(deck::stateName(r.state));
    m["fault"] = r.fault;
    m["faultName"] = QString::fromUtf8(deck::faultName(r.fault));
    m["stopReason"] = r.stopReason;
    m["stopName"] = QString::fromUtf8(deck::faultName(r.stopReason));
    m["faultServo"] = r.faultServo;
    m["legMask"] = r.legMask;
    m["text"] = QString::fromStdString(r.text);
    m["haveDetail"] = r.haveDetail;
    m["hotServo"] = r.hotServo;       m["hotTempC"] = r.hotTempC;
    m["packV"] = double(r.packV);     m["supplyV"] = double(r.supplyV);
    m["ripple"] = r.ripple;           m["solver"] = QString(QChar(r.solver));
    m["bodyHeightMm"] = r.bodyHeightMm; m["stepHeightMm"] = r.stepHeightMm;
    m["stanceMm"] = r.stanceMm;       m["speedMmS"] = r.speedMmS;
    m["seqRunning"] = r.seqRunning;
    m["seq"] = QString::fromStdString(r.seqName) + QStringLiteral(" %1/%2").arg(r.seqStep).arg(r.seqCount);
    m["haveAttitude"] = r.haveAttitude;
    m["rollDeg"] = double(r.rollRad) * 57.29578; m["pitchDeg"] = double(r.pitchRad) * 57.29578;
    // age of the last status line: stale information is red on the screen
    const qint64 nowMs = g_clock.nsecsElapsed() / 1000000;
    m["statusAgeS"] = r.haveStatus ? double(nowMs - r.statusAtMs) / 1000.0 : -1.0;
    return m;
}

QVariantMap Backend::telemetry() const {
    QVariantMap m;
    m["uplinkLq"] = int(m_tel.crsf_uplink_lq);
    m["downlinkLq"] = int(m_tel.crsf_downlink_lq);
    m["uplinkRssi"] = -int(m_tel.crsf_uplink_rssi_1);
    m["downlinkRssi"] = -int(m_tel.crsf_downlink_rssi);
    m["uplinkSnr"] = int(m_tel.crsf_uplink_snr);
    m["rfMode"] = int(m_tel.crsf_rf_mode);
    m["txPower"] = int(m_tel.crsf_uplink_tx_power);
    m["batteryV"] = m_tel.battery_voltage_mv / 1000.0;
    m["flightMode"] = QString::fromStdString(m_tel.flight_mode);
    m["frames"] = int(m_tel.crsf_frames_ok);
    m["crcErrors"] = int(m_tel.crsf_crc_errors);
    m["linkValid"] = m_tel.crsf_link_valid;
    return m;
}
