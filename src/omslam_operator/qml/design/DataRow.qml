import QtQuick
import QtQuick.Layouts
RowLayout {
    id:row
    property string label: ""
    property string value: ""
    property string unit: ""
    property color valueColor: "#E8EEF6"
    spacing:12; implicitHeight:28
    Text { text:row.label; color:"#9AAAC0"; font.pixelSize:12; Layout.fillWidth:true }
    Text { text:row.value; color:row.valueColor; font.pixelSize:13; font.weight:Font.Medium }
    Text { visible:row.unit!==""; text:row.unit; color:"#8294AC"; font.pixelSize:11 }
}
