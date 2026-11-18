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
    signal command(real v,real w,bool active)
    function release(){held=false;space=up=downKey=leftKey=rightKey=false;linear=angular=0;command(0,0,false)}
    function keys(){held=space;linear=space?((up?1:0)-(downKey&&reverseAllowed?1:0))*linearLimit:0;angular=space?((leftKey?1:0)-(rightKey?1:0))*angularLimit:0;command(linear,angular,held)}
    onActiveFocusChanged:if(!activeFocus)release()
    onEnabledChanged:if(!enabled)release()
    activeFocusOnTab:true
    Accessible.name:"직접 조종. 누른 채 움직이거나 Space와 방향키를 사용합니다."
    MouseArea {anchors.fill:parent;acceptedButtons:Qt.AllButtons;onWheel:function(wheel){wheel.accepted=true}}
    Keys.onPressed:function(event){if(event.isAutoRepeat)return;if(event.key===Qt.Key_Space)space=true;else if(event.key===Qt.Key_Up||event.key===Qt.Key_W)up=true;else if(event.key===Qt.Key_Down||event.key===Qt.Key_S)downKey=true;else if(event.key===Qt.Key_Left||event.key===Qt.Key_A)leftKey=true;else if(event.key===Qt.Key_Right||event.key===Qt.Key_D)rightKey=true;else return;event.accepted=true;keys()}
    Keys.onReleased:function(event){if(event.isAutoRepeat)return;if(event.key===Qt.Key_Space)space=false;else if(event.key===Qt.Key_Up||event.key===Qt.Key_W)up=false;else if(event.key===Qt.Key_Down||event.key===Qt.Key_S)downKey=false;else if(event.key===Qt.Key_Left||event.key===Qt.Key_A)leftKey=false;else if(event.key===Qt.Key_Right||event.key===Qt.Key_D)rightKey=false;else return;event.accepted=true;keys()}
    Rectangle {id:circle;anchors.left:parent.left;anchors.leftMargin:14;anchors.verticalCenter:parent.verticalCenter;width:96;height:96;radius:48;color:"#33274460";border.color:pad.activeFocus?"#6AA9FF":"#526C85";border.width:pad.activeFocus?2:1
        Rectangle {anchors.centerIn:parent;width:65;height:65;radius:33;color:"transparent";border.color:"#38516B"}
        Rectangle {x:33-pad.angular/Math.max(.01,pad.angularLimit)*22;y:33-pad.linear/Math.max(.01,pad.linearLimit)*22;width:30;height:30;radius:15;color:pad.held?"#76B2FF":"#3F628A";border.color:"#A1C7F5"}
        Text {anchors.top:parent.top;anchors.topMargin:5;anchors.horizontalCenter:parent.horizontalCenter;text:"↑";color:"#9BB4D1"}
        MouseArea {objectName:"joystickArea";anchors.fill:parent;cursorShape:pressed?Qt.ClosedHandCursor:Qt.OpenHandCursor
            function update(mouse){pad.linear=Math.max(pad.reverseAllowed?-1:0,Math.min(1,(height/2-mouse.y)/35))*pad.linearLimit;pad.angular=Math.max(-1,Math.min(1,(width/2-mouse.x)/35))*pad.angularLimit;pad.command(pad.linear,pad.angular,true)}
            onPressed:function(mouse){pad.forceActiveFocus();pad.held=true;update(mouse)}
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
