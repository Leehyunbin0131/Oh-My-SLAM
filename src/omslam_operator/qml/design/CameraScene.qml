import QtQuick

Item {
    id: scene
    property bool depth: false
    property bool compact: false
    readonly property var ui: scene.depth ? operatorModel.depthUi : operatorModel.rgbUi
    readonly property bool available: Boolean(depth?ui.depthAvailable:ui.rgbAvailable)
    readonly property bool fresh: Boolean(depth?ui.depthFresh:ui.rgbFresh)
    readonly property string statusText: (depth?ui.depthStatus:ui.rgbStatus) || "영상 수신 대기"
    readonly property int revision: (depth?ui.depthRevision:ui.rgbRevision) || 0
    clip: true
    Rectangle { anchors.fill: parent; color: "#101B28" }
    Image {
        objectName: scene.depth?"liveDepthImage":"liveRgbImage"
        anchors.fill: parent
        source: scene.visible && scene.available ? "image://camera/"+(scene.depth?"depth":"rgb")+"/"+scene.revision : ""
        cache: false; asynchronous: false; fillMode: Image.PreserveAspectFit
        opacity: scene.fresh ? 1 : .35
    }
    Text {
        anchors.centerIn: parent; width: parent.width-24
        visible: !scene.fresh; text: scene.statusText
        horizontalAlignment: Text.AlignHCenter; wrapMode: Text.Wrap
        color: scene.available?"#DDB56B":"#8FA8C4"; font.pixelSize: scene.compact?11:14
    }
    Rectangle {
        anchors.left: parent.left; anchors.top: parent.top; anchors.margins: scene.compact?6:10
        width: label.implicitWidth+14; height: 22; radius: 4; color: "#B5101821"
        Text { id: label; anchors.centerIn: parent; text: (scene.depth?"깊이":"RGB")+(scene.fresh?" · 실시간":""); color: "#E8EEF6"; font.pixelSize: 10 }
    }
}
