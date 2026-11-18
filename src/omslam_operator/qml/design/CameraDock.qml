import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: dock
    implicitWidth: 244
    implicitHeight: 244
    color: "#DE101B28"
    border.color: "#554C657F"
    radius: 10
    clip: true
    signal expandRequested(string kind)

    MouseArea {
        objectName: "cameraDockInputShield"
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        onWheel: function(wheel) { wheel.accepted = true }
    }
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 7
        RowLayout {
            Layout.fillWidth: true; Layout.preferredHeight: 17; Layout.maximumHeight: 17
            Text { Layout.minimumWidth: 0; Layout.fillWidth: true; text: "전방 카메라"; color: "#E8EEF6"; font.pixelSize: 12; font.weight: Font.DemiBold }
            Text { text: "클릭하여 확대"; color: "#9AAAC0"; font.pixelSize: 10 }
        }
        Button {
            id: rgbButton
            objectName: "cameraDockRgb"
            Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 62
            padding: 0; focusPolicy: Qt.StrongFocus
            Accessible.name: "RGB 영상 확대"
            background: Rectangle { color: "transparent" }
            contentItem: Item {
                clip: true
                CameraScene { anchors.fill: parent; depth: false; compact: true }
                Rectangle { anchors.fill: parent; color: "transparent"; border.color: rgbButton.activeFocus ? "#6AA9FF" : "#35465A"; border.width: rgbButton.activeFocus ? 2 : 1 }
            }
            onClicked: dock.expandRequested("rgb")
        }
        Button {
            id: depthButton
            objectName: "cameraDockDepth"
            Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 62
            padding: 0; focusPolicy: Qt.StrongFocus
            Accessible.name: "깊이 영상 확대"
            background: Rectangle { color: "transparent" }
            contentItem: Item {
                clip: true
                CameraScene { anchors.fill: parent; depth: true; compact: true }
                Rectangle { anchors.fill: parent; color: "transparent"; border.color: depthButton.activeFocus ? "#6AA9FF" : "#35465A"; border.width: depthButton.activeFocus ? 2 : 1 }
            }
            onClicked: dock.expandRequested("depth")
        }
    }
}
