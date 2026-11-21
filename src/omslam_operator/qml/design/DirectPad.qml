import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id:pad; objectName:"directPad"
    color:"#DE101B28";border.color:"#554C657F";radius:10
    property real linearLimit:0
    property real angularLimit:0
    property bool reverseAllowed:false
    property real linear:0
    property real angular:0
    property bool held:false
    property bool up:false
    property bool downKey:false
    property bool leftKey:false
    property bool rightKey:false
    property bool space:false
    property bool reverseNotice: false
    signal command(real v,real w,bool active)
    signal reverseDenied()
    function release() {
        held = false
        space = up = downKey = leftKey = rightKey = false
        linear = angular = 0
        reverseNotice = false
        command(0, 0, false)
    }
    function drive(forward, turn, pressed) {
        var denied = pressed && forward < 0 && !reverseAllowed
        if (denied && !reverseNotice) reverseDenied()
        reverseNotice = denied
        linear = pressed && !denied ? forward * linearLimit : 0
        angular = pressed ? turn * angularLimit : 0
        held = pressed && !(denied && turn === 0)
        command(linear, angular, held)
    }
    function keyEvent(event, pressed) {
        var key = event.key
        var field
        if (key === Qt.Key_Space) field = "space"
        else if (key === Qt.Key_Up || key === Qt.Key_W) field = "up"
        else if (key === Qt.Key_Down || key === Qt.Key_S) field = "downKey"
        else if (key === Qt.Key_Left || key === Qt.Key_A) field = "leftKey"
        else if (key === Qt.Key_Right || key === Qt.Key_D) field = "rightKey"
        else return
        event.accepted = true
        if (event.isAutoRepeat || stick.pressed) return
        pad[field] = pressed
        var forward = Number(up) - Number(downKey)
        var turn = Number(leftKey) - Number(rightKey)
        drive(forward, turn, space && (forward !== 0 || turn !== 0))
    }
    onActiveFocusChanged: if (!activeFocus) release()
    onEnabledChanged: if (!enabled) release()
    activeFocusOnTab: true
    Accessible.name: "직접 조종. 누른 채 움직이거나 Space와 방향키를 사용합니다."
    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        onPressed: pad.forceActiveFocus()
        onWheel: function(wheel) { wheel.accepted = true }
    }
    Keys.onPressed: function(event) { keyEvent(event, true) }
    Keys.onReleased: function(event) { keyEvent(event, false) }
    Rectangle {id:circle;anchors.left:parent.left;anchors.leftMargin:14;anchors.verticalCenter:parent.verticalCenter;width:96;height:96;radius:48;color:"#33274460";border.color:pad.activeFocus?"#6AA9FF":"#526C85";border.width:pad.activeFocus?2:1
        Rectangle {anchors.centerIn:parent;width:65;height:65;radius:33;color:"transparent";border.color:"#38516B"}
        Rectangle {x:33-pad.angular/Math.max(.01,pad.angularLimit)*22;y:33-pad.linear/Math.max(.01,pad.linearLimit)*22;width:30;height:30;radius:15;color:pad.held?"#76B2FF":"#3F628A";border.color:"#A1C7F5"}
        Text {anchors.top:parent.top;anchors.topMargin:5;anchors.horizontalCenter:parent.horizontalCenter;text:"↑";color:"#9BB4D1"}
        MouseArea {id:stick;objectName:"joystickArea";anchors.fill:parent;cursorShape:pressed?Qt.ClosedHandCursor:Qt.OpenHandCursor
            function update(mouse) {
                var forward = Math.max(-1, Math.min(1, (height / 2 - mouse.y) / 35))
                var turn = Math.max(-1, Math.min(1, (width / 2 - mouse.x) / 35))
                pad.drive(forward, turn, true)
            }
            onPressed: function(mouse) { pad.forceActiveFocus(); update(mouse) }
            onPositionChanged:function(mouse){if(pressed)update(mouse)}
            onReleased:pad.release()
            onCanceled:pad.release()
        }
    }
    ColumnLayout {anchors.left:circle.right;anchors.right:parent.right;anchors.leftMargin:12;anchors.rightMargin:12;anchors.verticalCenter:parent.verticalCenter;spacing:6
        Text {text:"직접 조종";color:"#E8EEF6";font.pixelSize:13;font.weight:Font.DemiBold}
        Text {text:"누른 채 움직이기";color:"#A1B5CE";font.pixelSize:11}
        Text {text:"Space + 방향키";color:"#8099B6";font.pixelSize:10}
        Text {text:pad.linear.toFixed(2)+" m/s";color:pad.held?"#91DCC5":"#8199B5";font.pixelSize:13;Layout.topMargin:4}
    }
}
