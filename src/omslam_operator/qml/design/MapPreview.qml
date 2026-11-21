import QtQuick
import OhMyRobot 1.0

Item {
    id: map
    objectName: "designMap"
    clip: true
    property var goals: operatorModel.goals
    property int selectedIndex: -1
    property bool adding: false
    property bool drawingKeepout: false
    readonly property bool followingRobot: nativeMap.followEnabled
    property bool editable: true
    property bool layerScan: true
    property bool layerCost: false
    property bool layerMemory: true
    property real leftInset: 288
    property real rightInset: 268
    property real bottomInset: 226
    signal pointPlaced(real x, real y, real yaw)
    signal keepoutPlaced(real x1, real y1, real x2, real y2)
    signal editCanceled()
    signal goalMoved(int index, real x, real y, real yaw)
    signal goalSelected(int index)
    function fit() { nativeMap.fit() }
    function followRobot(value) { nativeMap.followRobot(value) }
    function cancelGesture() { mapInput.dragIndex=-1; mapInput.rotating=false; mapInput.drawing=false }
    function syncViewport() { if(nativeMap)nativeMap.setViewport(leftInset, rightInset, bottomInset, 88) }
    function hitGoal(x, y, arrow) {
        for (var i=goals.length-1; i>=0; i--) {
            var goal=goals[i], p=nativeMap.screenPoint(goal.x, goal.y)
            var gx=p.x+(arrow?Math.cos(goal.yaw)*32:0)
            var gy=p.y-(arrow?Math.sin(goal.yaw)*32:0)
            if (Math.hypot(gx-x,gy-y)<(arrow?11:20)) return i
        }
        return -1
    }
    onLeftInsetChanged: syncViewport()
    onRightInsetChanged: syncViewport()
    onBottomInsetChanged: syncViewport()
    onLayerScanChanged: nativeMap.setLayer("scan", layerScan)
    onLayerMemoryChanged: nativeMap.setLayer("memory", layerMemory)
    onLayerCostChanged: { nativeMap.setLayer("global", layerCost); nativeMap.setLayer("local", layerCost) }
    Component.onCompleted: syncViewport()
    Accessible.name: "실시간 지도. +를 누른 뒤 위치를 클릭하고 드래그하여 도착 방향을 정합니다."
    NativeMap {
        id: nativeMap
        objectName: "nativeMap"
        anchors.fill: parent
        source: operatorModel
        selectedIndex: map.selectedIndex
    }
    MouseArea {
        id: mapInput
        objectName: "mapInput"
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        cursorShape: map.adding || map.drawingKeepout ? Qt.CrossCursor : pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor
        property real startX: 0
        property real startY: 0
        property real lastX: 0
        property real lastY: 0
        property int dragIndex: -1
        property bool rotating: false
        property bool drawing: false
        property real endX: 0
        property real endY: 0
        onPressed: function(mouse) {
            forceActiveFocus()
            if (mouse.button===Qt.RightButton) { map.cancelGesture(); map.editCanceled(); return }
            startX=lastX=endX=mouse.x; startY=lastY=endY=mouse.y
            drawing=map.drawingKeepout && map.editable
            if (!map.adding && !map.drawingKeepout && map.editable) {
                dragIndex=map.hitGoal(mouse.x,mouse.y,true); rotating=dragIndex>=0
                if (!rotating) dragIndex=map.hitGoal(mouse.x,mouse.y,false)
                if (dragIndex>=0) map.goalSelected(dragIndex)
            }
        }
        onPositionChanged: function(mouse) {
            if (!pressed || !(pressedButtons & Qt.LeftButton)) return
            endX=mouse.x; endY=mouse.y
            if (map.adding || map.drawingKeepout) return
            if (dragIndex<0) nativeMap.pan(mouse.x-lastX,mouse.y-lastY)
            lastX=mouse.x; lastY=mouse.y
        }
        onReleased: function(mouse) {
            if (mouse.button!==Qt.LeftButton) return
            if (dragIndex>=0 && dragIndex<map.goals.length && map.editable) {
                var goal=map.goals[dragIndex], p=nativeMap.screenPoint(goal.x,goal.y)
                if (rotating) map.goalMoved(dragIndex,goal.x,goal.y,Math.atan2(p.y-mouse.y,mouse.x-p.x))
                else { var world=nativeMap.worldPoint(mouse.x,mouse.y); map.goalMoved(dragIndex,world.x,world.y,goal.yaw) }
            } else if (drawing && map.drawingKeepout && map.editable) {
                if (Math.hypot(mouse.x-startX,mouse.y-startY)>=6) {
                    var a=nativeMap.worldPoint(startX,startY), b=nativeMap.worldPoint(mouse.x,mouse.y)
                    map.keepoutPlaced(a.x,a.y,b.x,b.y)
                }
            } else if (map.adding && map.editable) {
                var world=nativeMap.worldPoint(startX,startY)
                var yaw=Math.hypot(mouse.x-startX,mouse.y-startY)>6 ? Math.atan2(startY-mouse.y,mouse.x-startX) : 0
                map.pointPlaced(world.x,world.y,yaw)
            }
            dragIndex=-1; rotating=false; drawing=false
        }
        onCanceled: map.cancelGesture()
        onWheel: function(wheel) {
            if (wheel.angleDelta.y!==0 && !drawing && !map.adding) nativeMap.zoom(wheel.angleDelta.y>0?1.12:1/1.12)
        }
        Item {
            visible: mapInput.pressed && !mapInput.drawing && map.editable &&
                     (map.adding || mapInput.dragIndex >= 0) && Math.hypot(mapInput.endX-mapInput.startX,mapInput.endY-mapInput.startY)>3
            property var goal: mapInput.dragIndex>=0 ? map.goals[mapInput.dragIndex] : null
            property point anchor: goal && mapInput.rotating ? nativeMap.screenPoint(goal.x,goal.y) :
                                   goal ? Qt.point(mapInput.endX,mapInput.endY) : Qt.point(mapInput.startX,mapInput.startY)
            x: anchor.x
            y: anchor.y
            rotation: goal && !mapInput.rotating ? -goal.yaw*180/Math.PI :
                      Math.atan2(mapInput.endY-y,mapInput.endX-x)*180/Math.PI
            Rectangle { x:-7;y:-7;width:14;height:14;radius:7;color:"#8bc4ff" }
            Rectangle { x:0;y:-1.5;width:32;height:3;color:"#8bc4ff" }
            Rectangle { x:24;y:-4;width:9;height:3;rotation:35;color:"#8bc4ff" }
            Rectangle { x:24;y:1;width:9;height:3;rotation:-35;color:"#8bc4ff" }
        }
        Rectangle {
            visible: mapInput.drawing && map.drawingKeepout
            x: Math.min(mapInput.startX,mapInput.endX); y: Math.min(mapInput.startY,mapInput.endY)
            width: Math.abs(mapInput.endX-mapInput.startX); height: Math.abs(mapInput.endY-mapInput.startY)
            color: "#45EE6C78"; border.color: "#F7949D"; border.width: 2
        }
    }
}
