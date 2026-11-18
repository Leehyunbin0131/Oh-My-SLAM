import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: status
    property string stateCode: "not_connected"
    property string detail: ""
    property string reason: ""
    readonly property var messages: ({
        not_connected: "로봇 미연결 · ROS 연결을 확인하세요",
        checking: "이동 준비 상태를 확인 중입니다",
        ready: "이동 가능",
        disconnected: "로봇 연결 끊김 · 연결을 확인하세요",
        map_required: "지도를 불러오거나 새로 작성하세요",
        pose_required: "현재 위치를 지정하세요",
        no_path: "목적지까지 경로가 없습니다 · 목적지를 바꿔보세요",
        blocked: "이동할 수 없습니다 · 시스템 로그를 확인하세요"
    })
    readonly property string message: messages[stateCode] || messages.checking
    readonly property color statusColor: stateCode === "ready" ? "#63CFB1"
        : stateCode === "disconnected" || stateCode === "blocked" ? "#E98087"
        : stateCode === "checking" || !messages[stateCode] ? "#9AAAC0" : "#DDB56B"
    implicitHeight: 28
    implicitWidth: statusText.implicitWidth + 18
    Accessible.role: Accessible.StaticText
    Accessible.name: message

    RowLayout {
        anchors.fill: parent
        spacing: 9
        Rectangle { width: 7; height: 7; radius: 4; color: status.statusColor }
        Text {
            id: statusText
            objectName: "movementStatusText"
            Layout.fillWidth: true
            Layout.minimumWidth: 0
            text: status.message
            color: status.statusColor
            font.pixelSize: 12
            elide: Text.ElideRight
        }
    }
    HoverHandler { id: hover }
    ToolTip.visible: hover.hovered && statusText.truncated
    ToolTip.text: message
}
