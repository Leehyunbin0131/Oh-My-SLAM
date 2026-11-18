import QtQuick
import QtQuick.Layouts

Rectangle {
    id: pill
    property string text: ""
    property string tone: "neutral"
    property color ink: tone==="good" ? "#63CFB1" : tone==="warning" ? "#DDB56B" : tone==="danger" ? "#E98087" : tone==="blue" ? "#91BDFF" : "#AAB8CB"
    implicitWidth: content.implicitWidth+18; implicitHeight: 26
    color: tone==="good" ? "#173930" : tone==="warning" ? "#3C3321" : tone==="danger" ? "#41252C" : tone==="blue" ? "#213852" : "#26313F"
    radius: 5
    RowLayout { id:content; anchors.centerIn:parent; spacing:6
        Rectangle { width:5; height:5; radius:3; color:pill.ink }
        Text { text:pill.text; color:pill.ink; font.pixelSize:11; font.weight:Font.Medium }
    }
}
