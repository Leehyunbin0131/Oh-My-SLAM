import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: window
    width: 1440; height: 900; minimumWidth: 1280; minimumHeight: 800
    visible: true; title: "Oh My Robot · ROS Console"
    color: "#10151D"; font.family: "Noto Sans CJK KR"; font.pixelSize: 13
    palette.window: "#10151D"; palette.windowText: "#E8EEF6"; palette.text: "#E8EEF6"
    palette.base: "#151E2A"; palette.button: "#1D2938"; palette.buttonText: "#E8EEF6"
    palette.highlight: "#6AA9FF"; palette.highlightedText: "#10151D"
    property int page: 0
    property string notification: ""
    readonly property var ui: operatorModel.controlUi
    onPageChanged: { if(workspace)workspace.releaseManual(); operatorModel.setPage(page) }
    onActiveChanged: { operatorModel.setActive(active); if (!active&&workspace) workspace.releaseManual() }
    onVisibilityChanged: {
        if (window.visibility === Window.Hidden || window.visibility === Window.Minimized) {
            if (workspace) workspace.releaseManual()
            operatorModel.clearInput()
        }
    }
    onClosing: { workspace.releaseManual(); operatorModel.clearInput() }
    function notify(message) { notification=message; toastTimer.restart() }
    Connections { target: operatorModel; function onNotice(message) { window.notify(message) } }
    Timer { id: toastTimer; interval: 3600; onTriggered: window.notification="" }
    header: Rectangle {
        height: 58; color: "#131D29"
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: "#304053" }
        RowLayout {
            anchors.fill: parent; anchors.leftMargin: 20; anchors.rightMargin: 20; spacing: 14
            Icon { name: "robot"; ink: "#91BDFF"; Layout.preferredWidth: 25; Layout.preferredHeight: 25 }
            Text { text: "Oh My Robot"; color: "#E8EEF6"; font.pixelSize: 18; font.weight: Font.DemiBold }
            Rectangle { width: 1; height: 20; color: "#3A4A60"; Layout.leftMargin: 5; Layout.rightMargin: 5 }
            MovementStatus { objectName: "globalStatus"; stateCode: window.ui.movementState || "not_connected"; detail: window.ui.statusText || ""; reason: window.ui.reason || ""; Layout.fillWidth: true; Layout.minimumWidth: 0 }
            Text { text: window.ui.mapName || ""; color: "#8BA1BC"; font.pixelSize: 11; elide: Text.ElideRight; Layout.maximumWidth: 160 }
            Rectangle {
                objectName: "pathDebugStatus"; visible: Boolean(window.ui.pathDebugEnabled)
                implicitWidth: debugLabel.implicitWidth + 20; implicitHeight: 30; radius: 5
                color: window.ui.pathDebugState === "recording" ? "#193E37" : "#553829"
                Text { id: debugLabel; anchors.centerIn: parent; text: window.ui.pathDebugText || ""
                    color: window.ui.pathDebugState === "recording" ? "#8BDFC5" : "#F5BC8A"; font.pixelSize: 11 }
                HoverHandler { id: debugHover }
                ToolTip.visible: debugHover.hovered
                ToolTip.text: window.ui.pathDebugDetail || ""
            }
            ActionButton { objectName: "logsButton"; text: window.page===0?"시스템 로그":"지도"; iconName: window.page===0?"system":"map"; kind: "ghost"; onClicked: window.page=window.page===0?1:0 }
            ActionButton { objectName: "stopButton"; text: "정지"; iconName: "stop"; kind: "danger"; implicitWidth: 94; onClicked: { workspace.stopInput(); operatorModel.stop() } }
        }
    }
    StackLayout {
        anchors.fill: parent; currentIndex: window.page
        DrivingPage { id: workspace; objectName: "workspace"; onNotify: function(message) { window.notify(message) } }
        LogsPage { objectName: "logsPage"; events: operatorModel.logEntries }
    }
    Rectangle {
        visible: window.notification!==""; anchors.horizontalCenter: parent.horizontalCenter; anchors.top: parent.top; anchors.topMargin: 20
        width: Math.min(720,messageText.implicitWidth+36); height: messageText.implicitHeight+24; radius: 8; color: "#EF29405C"; border.color: "#55799E"; z: 100
        Text { id: messageText; anchors.centerIn: parent; width: Math.min(684,implicitWidth); text: window.notification; color: "#E8EEF6"; font.pixelSize: 13; wrapMode: Text.Wrap }
    }
}
