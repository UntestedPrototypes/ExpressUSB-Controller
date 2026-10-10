import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: win
    visible: true
    width: 1280
    height: 800
    title: "DeckController"
    color: "#12151d"

    readonly property var inp: backend.input
    readonly property var ctl: backend.control
    readonly property var tel: backend.telemetry
    readonly property var rob: backend.robot
    readonly property bool stale: rob.known && rob.statusAgeS > 6
    readonly property var pageNames: ["Main", "Channels", "Log", "Settings"]

    function pct(v) { return Math.round(v * 100) + " %" }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        // ---- status bar
        RowLayout {
            Layout.fillWidth: true
            spacing: 10
            StatusPill {
                text: backend.estop ? "E-STOP" : (backend.armed ? "ARMED" : (backend.armProgress > 0 ? "ARMING" : "DISARMED"))
                tone: backend.estop ? "#d32f2f" : (backend.armed ? "#2e7d32" : (backend.armProgress > 0 ? "#f9a825" : "#455064"))
            }
            StatusPill {
                text: backend.linkState
                tone: backend.linkState === "Link up" ? "#2e7d32" : (backend.linkState === "Port open" ? "#f9a825" : "#455064")
            }
            StatusPill {
                text: "UL " + tel.uplinkLq + " %  DL " + tel.downlinkLq + " %"
                tone: tel.uplinkLq > 70 ? "#2e7d32" : (tel.uplinkLq > 0 ? "#f9a825" : "#455064")
            }
            StatusPill {
                text: !rob.known ? "Robot: no data" : (rob.posture + (stale ? " (old)" : ""))
                tone: !rob.known || stale ? "#455064" : (rob.postureCode === "X" ? "#d32f2f" : (rob.postureCode === "I" ? "#2e7d32" : "#26599a"))
            }
            StatusPill {
                text: (rob.haveDetail && rob.packV > 0 ? rob.packV.toFixed(1) : (tel.batteryV > 0 ? tel.batteryV.toFixed(1) : "--")) + " V"
                tone: (rob.packV > 0 && rob.packV < 10.5) ? "#d32f2f" : "#3a4150"
            }
            StatusPill {
                visible: rob.haveDetail && rob.hotTempC > 0
                text: "S" + rob.hotServo + " " + rob.hotTempC + " C"
                tone: rob.hotTempC >= 65 ? "#d32f2f" : (rob.hotTempC >= 55 ? "#f9a825" : "#3a4150")
            }
            StatusPill {
                text: backend.controllerConnected ? backend.controllerName : "no controller"
                tone: backend.controllerConnected ? "#3a4150" : "#d32f2f"
            }
            Item { Layout.fillWidth: true }
            StatusPill { text: win.pageNames[backend.page]; tone: "#26324a" }
        }

        // arm / reset progress
        Rectangle {
            Layout.fillWidth: true
            height: 26
            radius: 6
            color: "#1d2230"
            visible: backend.armProgress > 0 || backend.resetProgress > 0
            Rectangle {
                height: parent.height
                radius: 6
                width: parent.width * Math.max(backend.armProgress, backend.resetProgress)
                color: backend.resetProgress > 0 ? "#ef6c00" : "#f9a825"
            }
            Text {
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 16
                text: backend.resetProgress > 0 ? "Hold Menu: reset the emergency stop" : "Hold A: arm"
            }
        }

        // fault banner
        Rectangle {
            Layout.fillWidth: true
            height: 34
            radius: 6
            color: "#b71c1c"
            visible: rob.known && (rob.fault > 0 || rob.stopReason > 0)
            Text {
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 18
                font.bold: true
                text: (rob.fault > 0 ? "FAULT: " + rob.faultName : "STOPPING: " + rob.stopName)
                      + (rob.faultServo > 0 ? "   servo " + rob.faultServo : "") + (rob.text.length ? "   " + rob.text : "")
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: backend.page

            // ---- page 0: main
            ColumnLayout {
                spacing: 16
                RowLayout {
                    spacing: 24
                    StickView { x01: inp.leftX; y01: inp.leftY; caption: "walk" }
                    PadView { down: inp.lpDown; px: inp.lpX; py: inp.lpY; caption: "stance" }
                    Item { Layout.fillWidth: true }
                    PadView { down: inp.rpDown; px: inp.rpX; py: inp.rpY; caption: "tilt" }
                    StickView { x01: inp.rightX; y01: inp.rightY; caption: "yaw / height" }
                }
                GridLayout {
                    columns: 4
                    columnSpacing: 30
                    rowSpacing: 8
                    Repeater {
                        model: [
                            ["Forward", pct(ctl.forward)],
                            ["Sideways", pct(ctl.strafe)],
                            ["Yaw rate", pct(ctl.turn)],
                            ["Body height", pct((ctl.height + 1) / 2)],
                            ["Roll", pct(ctl.roll) + (ctl.tiltRecorded ? "  (recorded)" : "")],
                            ["Pitch", pct(ctl.pitch)],
                            ["Stance", pct(ctl.stance) + (ctl.stanceRecorded ? "  (recorded)" : "")],
                            ["Gait", ctl.ripple ? "Ripple" : "Wave"],
                            ["Solver", ["Analytic", "Auto", "Numeric"][ctl.solver]],
                            ["Step height", pct((ctl.stepHeight + 1) / 2)],
                            ["Robot state", rob.known ? rob.state : "--"],
                            ["Robot height", rob.haveDetail ? rob.bodyHeightMm + " mm" : "--"],
                            ["Robot stance", rob.haveDetail ? rob.stanceMm + " mm" : "--"],
                            ["Step height", rob.haveDetail ? rob.stepHeightMm + " mm" : "--"],
                            ["Robot speed", rob.haveDetail ? rob.speedMmS + " mm/s" : "--"],
                            ["Robot gait / IK", rob.haveDetail ? (rob.ripple ? "Ripple" : "Wave") + " / " + ({"A": "analytic", "M": "auto", "N": "numeric"}[rob.solver]) : "--"],
                            ["Robot tilt", rob.haveAttitude ? rob.rollDeg.toFixed(1) + " / " + rob.pitchDeg.toFixed(1) + " deg" : "--"],
                            ["Sequence", rob.seqRunning ? rob.seq : "--"],
                            ["Supply", rob.haveDetail ? rob.supplyV.toFixed(1) + " V" : "--"]
                        ]
                        delegate: Column {
                            Text { text: modelData[0]; color: "#9aa3b8"; font.pixelSize: 14 }
                            Text { text: modelData[1]; color: "white"; font.pixelSize: 22 }
                        }
                    }
                }
                Item { Layout.fillHeight: true }
                RowLayout {
                    Button {
                        text: "EMERGENCY STOP"
                        onClicked: backend.emergencyStop()
                        implicitHeight: 60
                        implicitWidth: 260
                        background: Rectangle { color: "#d32f2f"; radius: 8 }
                        contentItem: Text {
                            text: parent.text
                            color: "white"
                            font.bold: true
                            font.pixelSize: 22
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    Button { text: "Disarm"; onClicked: backend.disarm(); implicitHeight: 60; implicitWidth: 160 }
                }
            }

            // ---- page 1: channels
            GridLayout {
                columns: 2
                columnSpacing: 40
                Repeater {
                    model: backend.channels
                    delegate: RowLayout {
                        Layout.preferredWidth: 560
                        Text { text: "CH" + (index + 1); color: "#9aa3b8"; font.pixelSize: 18; Layout.preferredWidth: 60 }
                        Rectangle {
                            Layout.fillWidth: true
                            height: 22
                            radius: 4
                            color: "#1d2230"
                            Rectangle {
                                height: parent.height
                                radius: 4
                                color: "#4fc3f7"
                                width: parent.width * Math.max(0, Math.min(1, (modelData - 988) / 1024))
                            }
                        }
                        Text { text: modelData + " us"; color: "white"; font.pixelSize: 18; Layout.preferredWidth: 90 }
                    }
                }
            }

            // ---- page 2: log
            ListView {
                clip: true
                model: backend.log
                delegate: Text { text: modelData; color: "white"; font.pixelSize: 16; width: ListView.view.width; elide: Text.ElideRight }
            }

            // ---- page 3: settings
            ColumnLayout {
                spacing: 14
                Text { text: "Radio link (TX module)"; color: "white"; font.pixelSize: 22 }
                RowLayout {
                    ComboBox { id: portBox; model: backend.ports; Layout.preferredWidth: 300 }
                    ComboBox { id: modeBox; model: ["crsf", "mavlink"]; Layout.preferredWidth: 160 }
                    Button { text: "Refresh"; onClicked: backend.refreshPorts() }
                    Button {
                        text: "Connect"
                        enabled: portBox.currentText.length > 0
                        onClicked: backend.connectLink(portBox.currentText, modeBox.currentText)
                    }
                    Button { text: "Disconnect"; onClicked: backend.disconnectLink() }
                }
                Text { text: "Touchpads found: " + backend.touchpads; color: "#9aa3b8"; font.pixelSize: 16 }
                Item { Layout.fillHeight: true }
            }
        }

        Text {
            Layout.fillWidth: true
            color: "#6d7790"
            font.pixelSize: 14
            text: "A hold 2 s: arm   A: disarm   B: EMERGENCY STOP   Menu hold 2 s: reset E-stop   View: next page"
        }
    }
}
