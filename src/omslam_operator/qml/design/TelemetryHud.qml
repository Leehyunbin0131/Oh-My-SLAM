import QtQuick
import QtQuick.Layouts

Rectangle {
    id: hud
    implicitHeight: 124
    readonly property var ui: operatorModel.telemetryUi
    readonly property var resourceData: operatorModel.resourcesUi
    color: "#DE101B28"
    border.color: "#554C657F"
    radius: 10
    clip: true

    component Label: Text {
        color: "#9AAAC0"
        font.pixelSize: 11
        elide: Text.ElideRight
    }
    component Title: Text {
        color: "#E8EEF6"
        font.pixelSize: 12
        font.weight: Font.DemiBold
    }
    component Reading: ColumnLayout {
        property string label: ""
        property string value: ""
        property color accent: "#E8EEF6"
        spacing: 4
        Label { text: parent.label }
        Text { text: parent.value; color: parent.accent; font.pixelSize: 25; font.weight: Font.Medium }
    }

    MouseArea {
        objectName: "hudInputShield"
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        onWheel: function(wheel) { wheel.accepted = true }
    }
    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 18
        ColumnLayout {
            Layout.minimumWidth: 0; Layout.preferredWidth: 1; Layout.fillWidth: true
            Layout.fillHeight: true; spacing: 10
            Title { objectName: "hudDriveTitle"; text: "모터 · RPM" }
            RowLayout {
                Layout.fillWidth: true; spacing: 30
                Reading { objectName: "leftRpmValue"; label: "왼쪽"; value: hud.ui.leftRpm || "—"; accent: "#B4D1FF" }
                Reading { objectName: "rightRpmValue"; label: "오른쪽"; value: hud.ui.rightRpm || "—"; accent: "#98E4D0" }
                Item { Layout.fillWidth: true }
            }
            Item { Layout.fillHeight: true }
        }
        Rectangle { Layout.preferredWidth: 1; Layout.fillHeight: true; color: "#35465A" }
        ColumnLayout {
            Layout.minimumWidth: 0; Layout.preferredWidth: 1; Layout.fillWidth: true
            Layout.fillHeight: true; spacing: 10
            Title { objectName: "hudImuTitle"; text: "자세 · 방향" }
            RowLayout {
                Layout.fillWidth: true; spacing: 24
                Reading { label: "롤"; value: hud.ui.roll && hud.ui.roll!=="—" ? hud.ui.roll+"°" : "—" }
                Reading { label: "피치"; value: hud.ui.pitch && hud.ui.pitch!=="—" ? hud.ui.pitch+"°" : "—" }
                Reading { objectName: "headingValue"; label: "방향 (지도)"; value: hud.ui.heading && hud.ui.heading!=="—" ? hud.ui.heading+"°" : "—"; accent: "#98E4D0" }
                Item { Layout.fillWidth: true }
            }
            Item { Layout.fillHeight: true }
        }
        Rectangle { Layout.preferredWidth: 1; Layout.fillHeight: true; color: "#35465A" }
        GridLayout {
            Layout.minimumWidth: 0; Layout.preferredWidth: 1; Layout.fillWidth: true
            Layout.fillHeight: true
            columns: 3; columnSpacing: 20; rowSpacing: 4
            Title { objectName: "hudHostTitle"; text: "시스템" }
            Label { text: "host"; color: "#6AA9FF"; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight }
            Label { text: "로봇"; color: "#63CFB1"; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight }
            Label { text: "CPU" }
            Text { text: hud.resourceData.hostCpu || "—"; color: "#E8EEF6"; font.pixelSize: 13; Layout.alignment: Qt.AlignRight }
            Text { text: hud.resourceData.robotCpu || "—"; color: "#E8EEF6"; font.pixelSize: 13; Layout.alignment: Qt.AlignRight }
            Label { text: "메모리" }
            Text { text: hud.resourceData.hostMemory || "—"; color: "#E8EEF6"; font.pixelSize: 12; Layout.alignment: Qt.AlignRight }
            Text { text: hud.resourceData.robotMemory || "—"; color: "#E8EEF6"; font.pixelSize: 12; Layout.alignment: Qt.AlignRight }
            Label { text: "센서 최고 온도" }
            Text { text: hud.resourceData.hostTemperature || "—"; color: "#E8EEF6"; font.pixelSize: 12; Layout.alignment: Qt.AlignRight }
            Text { text: hud.resourceData.robotTemperature || "—"; color: "#E8EEF6"; font.pixelSize: 12; Layout.alignment: Qt.AlignRight }
        }
    }
}
