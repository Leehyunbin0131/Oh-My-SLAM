import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Button {
    id: control
    property string kind: "secondary"
    property string iconName: ""
    property bool compact: false
    property bool selected: false
    implicitHeight: compact ? 32 : 40
    implicitWidth: contentItem.implicitWidth + (compact ? 20 : 28)
    padding: compact ? 8 : 12
    hoverEnabled: true
    font.pixelSize: compact ? 12 : 13
    font.weight: kind === "primary" ? Font.DemiBold : Font.Medium
    Accessible.name: text
    background: Rectangle {
        radius: 7
        color: control.kind==="primary" ? (control.down ? "#4189E8" : control.hovered ? "#83B8FF" : "#6AA9FF") : control.kind==="danger" ? (control.hovered ? "#B84C56" : "#A73C47") : control.selected ? "#263E5D" : control.kind==="ghost" ? (control.hovered ? "#243144" : "transparent") : control.hovered ? "#29384B" : "#1D2938"
        border.color: control.activeFocus ? "#A4CBFF" : control.selected ? "#6AA9FF" : control.kind==="secondary" ? "#35455B" : "transparent"
        border.width: control.activeFocus ? 2 : 1
        opacity: control.enabled ? 1 : .38
    }
    contentItem: RowLayout {
        spacing: 7
        Icon { visible: control.iconName!==""; name: control.iconName; ink: control.kind==="primary" ? "#101D30" : "#E8EEF6"; Layout.preferredWidth: 17; Layout.preferredHeight: 17 }
        Text { text: control.text; font: control.font; color: control.kind==="primary" ? "#101D30" : "#E8EEF6"; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true; opacity: control.enabled?1:.45 }
    }
}
