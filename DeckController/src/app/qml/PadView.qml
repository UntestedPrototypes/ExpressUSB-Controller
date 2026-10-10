import QtQuick

// A touchpad: rectangle with the finger. px, py in 0..1, y up.
Item {
    id: root
    property bool down: false
    property real px: 0.5
    property real py: 0.5
    property string caption: ""
    implicitWidth: 150
    implicitHeight: 175
    Rectangle {
        id: pad
        width: 140; height: 140; radius: 14
        anchors.horizontalCenter: parent.horizontalCenter
        color: root.down ? "#243247" : "#1d2230"; border.color: "#4a5268"; border.width: 2
        Rectangle {
            visible: root.down
            width: 26; height: 26; radius: 13; color: "#ffb74d"
            x: root.px * pad.width - 13
            y: (1 - root.py) * pad.height - 13
        }
    }
    Text { anchors.bottom: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter; text: root.caption; color: "#9aa3b8"; font.pixelSize: 16 }
}
