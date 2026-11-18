import QtQuick
import QtQuick.Layouts
RowLayout {
    property string text: ""
    property string detail: ""
    spacing:12
    Text { text:parent.text; color:"#E8EEF6"; font.pixelSize:15; font.weight:Font.DemiBold; Layout.fillWidth:true }
    Text { text:parent.detail; color:"#9AAAC0"; font.pixelSize:11 }
}
