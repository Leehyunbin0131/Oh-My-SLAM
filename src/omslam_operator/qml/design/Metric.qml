import QtQuick
import QtQuick.Layouts
ColumnLayout {
    id:metric
    property string label: ""
    property string value: ""
    property string unit: ""
    property string detail: ""
    property color accent: "#E8EEF6"
    spacing:6
    Text { text:metric.label; color:"#9AAAC0"; font.pixelSize:12 }
    RowLayout { spacing:5
        Text { text:metric.value; color:metric.accent; font.pixelSize:29; font.weight:Font.Medium }
        Text { text:metric.unit; color:"#9AAAC0"; font.pixelSize:12; Layout.alignment:Qt.AlignBottom; Layout.bottomMargin:5 }
    }
    Text { visible:text!==""; text:metric.detail; color:"#8798AE"; font.pixelSize:11 }
}
