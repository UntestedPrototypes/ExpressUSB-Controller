import QtQuick

// A rounded label with a colour state
Rectangle {
    id: root
    property string text: ""
    property color tone: "#3a4150"
    implicitWidth: label.implicitWidth + 28
    implicitHeight: 40
    radius: 20
    color: tone
    Text {
        id: label
        anchors.centerIn: parent
        text: root.text
        color: "white"
        font.pixelSize: 18
        font.bold: true
    }
}
