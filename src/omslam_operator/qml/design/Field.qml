import QtQuick
import QtQuick.Controls

TextField {
    id:field
    color:"#E8EEF6"; selectedTextColor:"#10151D"; selectionColor:"#6AA9FF"
    placeholderTextColor:"#71839B"; font.pixelSize:13; selectByMouse:true
    implicitHeight:40; leftPadding:12; rightPadding:12
    background:Rectangle { radius:6; color:"#101923"; border.color:field.activeFocus ? "#6AA9FF":"#35455B"; border.width:field.activeFocus?2:1 }
}
