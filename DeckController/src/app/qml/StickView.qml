import QtQuick

// A stick: circle with a dot. x01, y01 in -1..1, y up.
Item {
    id: root
    property real x01: 0
    property real y01: 0
    property string caption: ""
    implicitWidth: 150
    implicitHeight: 175
    Rectangle {
        id: ring
        width: 140; height: 140; radius: 70
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#1d2230"; border.color: "#4a5268"; border.width: 2
        Rectangle { width: 1; height: parent.height; anchors.centerIn: parent; color: "#33394a" }
        Rectangle { height: 1; width: parent.width; anchors.centerIn: parent; color: "#33394a" }
        Rectangle {
            width: 22; height: 22; radius: 11; color: "#4fc3f7"
            x: ring.width / 2 + root.x01 * 60 - 11
            y: ring.height / 2 - root.y01 * 60 - 11
        }
    }
    Text { anchors.bottom: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter; text: root.caption; color: "#9aa3b8"; font.pixelSize: 16 }
}
