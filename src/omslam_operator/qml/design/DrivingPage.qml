import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: page
    signal notify(string message)
    readonly property var ui: operatorModel.controlUi
    readonly property var telemetryData: operatorModel.telemetryUi
    readonly property var routePoints: operatorModel.goals
    readonly property int goalCount: routePoints.length
    readonly property bool moving: Boolean(ui.routeRunning)
    readonly property bool editable: !ui.routeBusy && !ui.operationPending && !ui.readOnly
    property bool addingGoal: false
    property bool addingKeepout: false
    readonly property var keepouts: operatorModel.keepoutZones
    readonly property bool keepoutEditable: editable && Boolean(ui.keepoutAvailable) && !ui.keepoutBusy
    property bool initialPosePending: false
    property int selectedGoal: -1
    property string cameraKind: ""
    property int editingGoal: -1
    property string mapPathOperation: "save"
    readonly property int activeGoal: {
        for (var i=0;i<routePoints.length;i++) if (routePoints[i].status==="이동 중") return i
        return -1
    }
    readonly property string statusText: addingKeepout?"금지 구역의 모서리에서 반대 모서리까지 드래그하세요":addingGoal?"지도에서 위치를 클릭하고 드래그해 도착 방향을 정하세요":initialPosePending?"지도에서 현재 위치를 클릭하고 바라보는 방향으로 드래그하세요":""
    ListModel { id: goalRows; objectName: "waypointModel" }
    function syncGoalRows() {
        if (!goalRows) return
        var structureChanged=goalRows.count!==routePoints.length
        if (!structureChanged) for (var i=0;i<routePoints.length;i++) if (goalRows.get(i).uid!==routePoints[i].uid) { structureChanged=true; break }
        if (structureChanged) goalRows.clear()
        for (var j=0;j<routePoints.length;j++) {
            var g=routePoints[j], row={uid:g.uid,gx:g.x,gy:g.y,heading:g.yaw*180/Math.PI,status:g.status}
            if (structureChanged) goalRows.append(row)
            else {
                var current=goalRows.get(j)
                if(current.gx!==row.gx)goalRows.setProperty(j,"gx",row.gx)
                if(current.gy!==row.gy)goalRows.setProperty(j,"gy",row.gy)
                if(current.heading!==row.heading)goalRows.setProperty(j,"heading",row.heading)
                if(current.status!==row.status)goalRows.setProperty(j,"status",row.status)
            }
        }
    }
    onRoutePointsChanged: syncGoalRows()
    Component.onCompleted: syncGoalRows()
    function releaseManual() { if(directPad)directPad.release() }
    function cancelMapEdit() { addingGoal=false; addingKeepout=false; initialPosePending=false; editingGoal=-1; map.cancelGesture() }
    function stopInput() { releaseManual(); cancelMapEdit() }
    function beginAdd() {
        var activate=!addingGoal; stopInput(); addingGoal=activate
        if (activate) { map.followRobot(false); forceActiveFocus() }
    }
    function beginKeepout() {
        stopInput(); addingKeepout=true; keepoutTools.close(); map.followRobot(false); forceActiveFocus()
    }
    function placeKeepout(x1,y1,x2,y2) {
        if (!keepoutEditable) return
        addingKeepout=false; operatorModel.addKeepout(x1,y1,x2,y2)
    }
    function placeGoal(x,y,yaw) {
        if (initialPosePending) { operatorModel.setInitialPose(x,y,yaw); initialPosePending=false; return }
        var before=goalCount
        operatorModel.addWaypoint(x,y,yaw)
        if (goalCount>before) { selectedGoal=goalCount-1; addingGoal=false; goalList.currentIndex=selectedGoal; goalList.positionViewAtIndex(selectedGoal,ListView.Contain) }
    }
    function deleteGoal(index) {
        if (index<0 || index>=goalCount || !editable) return
        editingGoal=-1; operatorModel.removeGoal(index); selectedGoal=Math.min(index,goalCount-1)
    }
    function reorderGoal(from,to) {
        to=Math.max(0,Math.min(goalCount-1,to))
        if (from===to || !editable) return
        editingGoal=-1; operatorModel.moveWaypoint(from,to); selectedGoal=to
        goalList.currentIndex=to; goalList.positionViewAtIndex(to,ListView.Contain)
    }
    function updateGoal(index,x,y,yaw) {
        if (index<0 || index>=goalCount || !editable || !isFinite(x) || !isFinite(y) || !isFinite(yaw)) return
        operatorModel.editWaypoint(index,x,y,yaw); selectedGoal=index
    }
    function editGoal(index) {
        if (!editable) return
        editingGoal=index; selectedGoal=index; var goal=routePoints[index]
        editX.text=goal.x.toFixed(2); editY.text=goal.y.toFixed(2); editYaw.text=(goal.yaw*180/Math.PI).toFixed(1)
    }
    function commitCoordinates() {
        if (editingGoal>=0 && editX.text.trim()!=="" && editY.text.trim()!=="" && editYaw.text.trim()!=="")
            updateGoal(editingGoal,Number(editX.text),Number(editY.text),Number(editYaw.text)*Math.PI/180)
    }
    function startRoute() { releaseManual(); editingGoal=-1; operatorModel.startRoute() }
    function manualCommand(v,w,pressed) {
        if (pressed) { addingGoal=false; addingKeepout=false; initialPosePending=false; map.cancelGesture() }
        operatorModel.manualIntent(v,w,pressed)
    }
    function openMapPath(operation) {
        stopInput(); mapPathOperation=operation
        mapPath.text=operation==="save" ? defaultMapDirectory+"/map_"+Qt.formatDateTime(new Date(),"yyyyMMdd_HHmmss") : ""
        mapPathDialog.open(); mapPath.forceActiveFocus()
    }
    onGoalCountChanged: { if (editingGoal>=goalCount) editingGoal=-1; if (selectedGoal>=goalCount) selectedGoal=goalCount-1 }
    onEditableChanged: if (!editable) cancelMapEdit()
    onKeepoutEditableChanged: if (!keepoutEditable) { addingKeepout=false; map.cancelGesture() }
    onVisibleChanged: if (!visible) stopInput()
    Keys.onEscapePressed: { stopInput(); cameraKind=""; mapTools.close(); keepoutTools.close(); mapPathDialog.close() }
    Shortcut {
        sequence: "Escape"; context: Qt.WindowShortcut
        enabled: page.visible && (page.addingGoal || page.initialPosePending || page.addingKeepout)
        onActivated: page.stopInput()
    }
    MapPreview {
        id: map; anchors.fill: parent
        goals: page.routePoints; selectedIndex: page.selectedGoal
        adding: page.addingGoal || page.initialPosePending; editable: page.editable
        drawingKeepout: page.addingKeepout
        onKeepoutPlaced: function(x1,y1,x2,y2) { page.placeKeepout(x1,y1,x2,y2) }
        onEditCanceled: page.stopInput()
        bottomInset: telemetry.height+70
        onPointPlaced: function(x,y,yaw) { page.placeGoal(x,y,yaw) }
        onGoalSelected: function(index) { page.selectedGoal=index; page.editingGoal=-1 }
        onGoalMoved: function(index,x,y,yaw) { page.updateGoal(index,x,y,yaw) }
    }
    Rectangle {
        visible: page.statusText!==""; anchors.horizontalCenter: parent.horizontalCenter; y: 78
        width: hintRow.implicitWidth+20; height: 38; radius: 7; color: "#ED29405C"; z: 10
        RowLayout { id: hintRow; anchors.centerIn: parent; spacing: 10
            Text { text: page.statusText; color: "#D9E6F6"; font.pixelSize: 12 }
            ActionButton { objectName:"cancelMapEditButton"; text:"취소 · Esc"; compact:true; kind:"ghost"; onClicked:page.stopInput() }
        }
    }
    Rectangle {
        id:goalPanel;objectName:"goalPanel";x:16;y:16;width:260
        height:Math.min(directPad.y-28,page.goalCount?220+page.goalCount*50:282)
        color:"#E5101B28";border.color:"#554C657F";radius:10
        MouseArea {anchors.fill:parent}
        ColumnLayout {anchors.fill:parent;anchors.margins:14;spacing:10
            RowLayout {Layout.fillWidth:true;spacing:8
                Text {text:"목적지";color:"#E8EEF6";font.pixelSize:16;font.weight:Font.DemiBold;Layout.fillWidth:true}
                Text {text:page.goalCount?page.goalCount+"개":"";color:"#8DA5C0";font.pixelSize:12}
                ActionButton {objectName:"addGoalButton";text:page.addingGoal?"취소":"+";compact:true;implicitWidth:page.addingGoal?54:34;kind:page.addingGoal?"primary":"secondary";Accessible.name:page.addingGoal?"목적지 추가 취소":"목적지 추가";ToolTip.visible:hovered;ToolTip.text:page.addingGoal?"추가 취소 (Esc 또는 우클릭)":"지도에 목적지 추가";enabled:page.editable&&Boolean(page.ui.mapReady);onClicked:page.beginAdd()}
            }
            Item {Layout.fillWidth:true;Layout.fillHeight:true
                Column {visible:page.goalCount===0;anchors.centerIn:parent;spacing:9
                    Text {text:page.addingGoal?"지도에서 위치를 선택하세요":"목적지를 추가하세요";color:"#CBD9EA";font.pixelSize:13;anchors.horizontalCenter:parent.horizontalCenter}
                    Text {text:"+ 누르고 지도 한 번 클릭";color:"#8BA4C0";font.pixelSize:11;anchors.horizontalCenter:parent.horizontalCenter}
                    Text {text:"드래그하면 도착 방향도 지정됩니다";color:"#718BA8";font.pixelSize:10;anchors.horizontalCenter:parent.horizontalCenter}
                }
                ListView {
                    id:goalList;objectName:"goalList";visible:page.goalCount>0;anchors.fill:parent;clip:true;model:goalRows;spacing:0
                    ScrollBar.vertical:ScrollBar{}
                    delegate:Rectangle {
                        id:row;required property int index;required property real gx;required property real gy;required property real heading
                        objectName:"goalRow"+index;width:goalList.width;height:50;radius:6
                        color:page.activeGoal===index?"#503A7B6C":page.selectedGoal===index?"#4338587D":"transparent"
                        RowLayout {anchors.fill:parent;spacing:5
                            Item {objectName:"reorderHandle"+row.index;Layout.preferredWidth:22;Layout.fillHeight:true;activeFocusOnTab:true
                                Accessible.role:Accessible.Button;enabled:page.editable;Accessible.name:"목적지 "+(row.index+1)+" 순서 변경. 위아래로 끌거나 방향키를 누르세요."
                                Canvas {anchors.centerIn:parent;width:12;height:18;onPaint:{var c=getContext("2d");c.reset();c.fillStyle="#7E9ABC";for(var x=0;x<2;x++)for(var y=0;y<3;y++){c.beginPath();c.arc(3+x*6,3+y*6,1.2,0,7);c.fill();}}}
                                Rectangle {anchors.fill:parent;radius:4;color:"transparent";border.color:parent.activeFocus?"#8DBBFF":"transparent"}
                                Keys.onUpPressed:page.reorderGoal(row.index,row.index-1)
                                Keys.onDownPressed:page.reorderGoal(row.index,row.index+1)
                                MouseArea {anchors.fill:parent;preventStealing:true;cursorShape:pressed?Qt.ClosedHandCursor:Qt.OpenHandCursor;property real startY
                                    onPressed:function(mouse){startY=mouse.y;parent.forceActiveFocus()}
                                    onReleased:function(mouse){page.reorderGoal(row.index,row.index+Math.round((mouse.y-startY)/50))}
                                }
                            }
                            Rectangle {width:23;height:23;radius:12;color:page.activeGoal===row.index?"#3C8A72":"#355C86";Text {anchors.centerIn:parent;text:row.index+1;color:"#D9EAFE";font.pixelSize:11}}
                            Button {objectName:"goalCoordinates"+row.index;Layout.fillWidth:true;Layout.fillHeight:true;hoverEnabled:true
                                background:Rectangle {radius:4;color:parent.hovered?"#253D58":"transparent";border.color:parent.activeFocus?"#8DBBFF":"transparent"}
                                contentItem:Column {spacing:2
                                    Text {text:row.gx.toFixed(2)+", "+row.gy.toFixed(2)+" m";color:"#D9E6F6";font.pixelSize:12}
                                    Text {text:"도착 방향 "+row.heading.toFixed(0)+"°";color:"#8FA8C4";font.pixelSize:10}
                                }
                                onClicked:page.editGoal(row.index)
                            }
                            ActionButton {objectName:"deleteGoal"+row.index;text:"−";kind:"ghost";compact:true;implicitWidth:27;Accessible.name:"목적지 "+(row.index+1)+" 삭제";enabled:page.editable;onClicked:page.deleteGoal(row.index)}
                        }
                    }
                }
            }
            Text {visible:page.goalCount>1;text:"손잡이를 끌어 순서 변경";color:"#7F99B6";font.pixelSize:10}
            Rectangle {Layout.fillWidth:true;height:1;color:"#35485E"}
            RowLayout {Layout.fillWidth:true;spacing:8
                Text {text:"RPM";color:"#A6B9D1";font.pixelSize:11}
                Slider {id:speed;objectName:"speedSlider";from:Number(page.ui.minRpm||1);to:Math.max(from,Number(page.ui.maxRpm||100));value:Number(page.ui.rpmLimit||from);stepSize:Number(page.ui.rpmStep||1);snapMode:Slider.SnapAlways;onMoved:operatorModel.setRpmLimit(value);enabled:!page.ui.readOnly&&!page.ui.ownerBusy&&Boolean(page.ui.controlFresh);Layout.fillWidth:true;implicitHeight:24
                    background:Rectangle {x:speed.leftPadding;y:speed.topPadding+speed.availableHeight/2-height/2;implicitHeight:3;width:speed.availableWidth;radius:2;color:"#425A74";Rectangle {width:speed.visualPosition*parent.width;height:parent.height;radius:2;color:"#6AA9FF"}}
                    handle:Rectangle {x:speed.leftPadding+speed.visualPosition*(speed.availableWidth-width);y:speed.topPadding+speed.availableHeight/2-height/2;width:13;height:13;radius:7;color:"#8EBDFF";border.color:speed.activeFocus?"#E8EEF6":"transparent"}
                    Accessible.name:"바퀴 RPM 상한"
                }
                Field {
                    id:rpmValue; objectName:"rpmLimitInput"; Layout.preferredWidth:65; implicitHeight:32
                    enabled:!page.ui.readOnly&&!page.ui.ownerBusy&&Boolean(page.ui.controlFresh); horizontalAlignment:Text.AlignRight
                    validator:DoubleValidator {bottom:Number(page.ui.minRpm||1);top:Number(page.ui.maxRpm||100);decimals:1;notation:DoubleValidator.StandardNotation}
                    Accessible.name:"바퀴 RPM 상한 직접 입력"
                    onEditingFinished: {
                        if (acceptableInput && text.trim()!=="") operatorModel.setRpmLimit(Number.fromLocaleString(Qt.locale(),text))
                    }
                    Binding {target:rpmValue;property:"text";value:Number(page.ui.rpmLimit||0).toLocaleString(Qt.locale(),'f',1);when:!rpmValue.activeFocus;restoreMode:Binding.RestoreNone}
                }
            }
            Text {text:"바퀴별 상한 · 최대 "+Number(page.ui.maxRpm||100).toFixed(0)+" RPM";color:"#8BA4C0";font.pixelSize:10}
            ActionButton {objectName:"moveButton";text:page.moving?"이동 중":"이동";iconName:"play";kind:"primary";Layout.fillWidth:true;implicitHeight:40;enabled:page.goalCount>0&&!page.addingGoal&&!page.addingKeepout&&page.editable&&Boolean(page.ui.mapReady)&&!page.ui.needsInitialPose;onClicked:page.startRoute()}
        }
    }
    DirectPad {id:directPad;x:16;width:260;height:130;anchors.bottom:telemetry.top;anchors.bottomMargin:14;linearLimit:Number(page.ui.linearLimit||0);angularLimit:Number(page.ui.angularLimit||0);reverseAllowed:Boolean(page.ui.reverseAllowed);enabled:!page.ui.readOnly&&!page.ui.ownerBusy&&Boolean(page.ui.controlFresh);onCommand:function(v,w,active){page.manualCommand(v,w,active)}}
    Rectangle {anchors.top:parent.top;anchors.topMargin:16;anchors.horizontalCenter:parent.horizontalCenter;implicitWidth:motionRow.implicitWidth+32;implicitHeight:50;radius:9;color:"#D8101B28";border.color:"#554C657F"
        MouseArea {anchors.fill:parent;acceptedButtons:Qt.AllButtons;onWheel:function(wheel){wheel.accepted=true}}
        RowLayout {id:motionRow;anchors.centerIn:parent;spacing:18
            Column {spacing:1;Text {text:"선속도";color:"#8FA8C3";font.pixelSize:10}Text {text:page.telemetryData.linearVelocity&&page.telemetryData.linearVelocity!=="—"?page.telemetryData.linearVelocity+" m/s":"—";color:"#8BDFC5";font.pixelSize:18;font.weight:Font.Medium}}
            Rectangle {width:1;height:24;color:"#41536A"}
            Column {spacing:1;Text {text:"각속도";color:"#8FA8C3";font.pixelSize:10}Text {text:page.telemetryData.angularVelocity&&page.telemetryData.angularVelocity!=="—"?page.telemetryData.angularVelocity+" rad/s":"—";color:"#D9E7F6";font.pixelSize:15}}
            Column {spacing:1;Text {text:"위치 x / y";color:"#8FA8C3";font.pixelSize:10}Text {text:page.telemetryData.position||"—";color:"#B8CCE4";font.pixelSize:12}}
        }
    }
    CameraDock {id:cameras;objectName:"cameraDock";width:244;height:implicitHeight;anchors.right:parent.right;anchors.top:parent.top;anchors.margins:16;onExpandRequested:function(kind){page.cameraKind=kind}}
    Row {anchors.right:parent.right;anchors.rightMargin:16;anchors.bottom:telemetry.top;anchors.bottomMargin:14;spacing:8
        ActionButton {objectName:"mapToolsButton";text:"지도";iconName:"folder";compact:true;onClicked:mapTools.open()}
        ActionButton {objectName:"keepoutToolsButton";text:"진입 금지";compact:true;onClicked:keepoutTools.open()}
        ActionButton {objectName:"followRobotButton";text:map.followingRobot?"따라가기 켜짐":"로봇 따라가기";selected:map.followingRobot;compact:true;onClicked:{page.stopInput();map.followRobot(!map.followingRobot)}}
        ActionButton {objectName:"mapFitButton";text:"전체 보기";iconName:"expand";compact:true;onClicked:{page.stopInput();map.fit()}}
    }
    TelemetryHud {id:telemetry;objectName:"telemetryHud";anchors.left:parent.left;anchors.right:parent.right;anchors.bottom:parent.bottom;anchors.margins:16;height:implicitHeight}

    Rectangle {visible:page.editingGoal>=0;objectName:"coordinateEditor";x:goalPanel.x+goalPanel.width+12;y:goalPanel.y+48;width:300;height:145;radius:9;color:"#F0192B3F";border.color:"#6C90B4";z:20
        MouseArea {anchors.fill:parent}
        ColumnLayout {anchors.fill:parent;anchors.margins:14;spacing:9
            RowLayout {Layout.fillWidth:true;Text {text:"목적지 "+(page.editingGoal+1)+" 좌표";color:"#E8EEF6";font.pixelSize:13;Layout.fillWidth:true}ActionButton {text:"닫기";compact:true;kind:"ghost";onClicked:page.editingGoal=-1}}
            RowLayout {spacing:7;Layout.fillWidth:true
                ColumnLayout {Layout.fillWidth:true;Text {text:"X (m)";color:"#9AAAC0";font.pixelSize:10}Field {id:editX;objectName:"editGoalX";Layout.fillWidth:true;Layout.preferredWidth:1;validator:DoubleValidator{} onEditingFinished:page.commitCoordinates()}}
                ColumnLayout {Layout.fillWidth:true;Text {text:"Y (m)";color:"#9AAAC0";font.pixelSize:10}Field {id:editY;objectName:"editGoalY";Layout.fillWidth:true;Layout.preferredWidth:1;validator:DoubleValidator{} onEditingFinished:page.commitCoordinates()}}
                ColumnLayout {Layout.fillWidth:true;Text {text:"방향 (°)";color:"#9AAAC0";font.pixelSize:10}Field {id:editYaw;objectName:"editGoalYaw";Layout.fillWidth:true;Layout.preferredWidth:1;validator:DoubleValidator{} onEditingFinished:page.commitCoordinates()}}
            }
        }
    }
    Popup {id:mapTools;objectName:"mapTools";x:page.width-width-16;y:page.height-telemetry.height-height-54;width:260;height:320;padding:14;closePolicy:Popup.CloseOnEscape|Popup.CloseOnPressOutside
        background:Rectangle {color:"#F0192B3F";radius:9;border.color:"#6382A3"}
        ColumnLayout {anchors.fill:parent;spacing:6
            ActionButton {objectName:"mapSaveButton";text:"지도 저장";kind:"ghost";compact:true;Layout.fillWidth:true;enabled:page.editable&&Boolean(page.ui.mapReady);onClicked:{mapTools.close();page.openMapPath("save")}}
            ActionButton {objectName:"mapLoadButton";text:"저장 지도 열기";kind:"ghost";compact:true;Layout.fillWidth:true;enabled:page.editable;onClicked:{mapTools.close();page.openMapPath("load")}}
            ActionButton {objectName:"newMapButton";text:"새 지도 작성";kind:"ghost";compact:true;Layout.fillWidth:true;enabled:page.editable;onClicked:{mapTools.close();page.releaseManual();operatorModel.newMap()}}
            ActionButton {objectName:"initialPoseButton";text:"현재 위치 맞추기";kind:"ghost";compact:true;Layout.fillWidth:true;enabled:page.editable&&Boolean(page.ui.mapReady);onClicked:{mapTools.close();page.releaseManual();page.stopInput();page.initialPosePending=true;map.followRobot(false);page.forceActiveFocus()}}
            Rectangle {height:1;Layout.fillWidth:true;color:"#3C526E"}
            CheckBox {text:"라이다";checked:map.layerScan;onToggled:map.layerScan=checked;implicitHeight:30}
            CheckBox {text:"장애물 기억";checked:map.layerMemory;onToggled:map.layerMemory=checked;implicitHeight:30}
            CheckBox {text:"비용지도";checked:map.layerCost;onToggled:map.layerCost=checked;implicitHeight:30}
        }
    }
    Popup {
        id:keepoutTools; objectName:"keepoutTools"; width:310; height:Math.min(page.height-130,245+Math.min(page.keepouts.length,5)*45)
        x:page.width-width-16; y:Math.max(12,page.height-telemetry.height-height-54); padding:14
        closePolicy:Popup.CloseOnEscape|Popup.CloseOnPressOutside; focus:true
        background:Rectangle {color:"#F0192B3F";radius:9;border.color:"#6382A3"}
        ColumnLayout {anchors.fill:parent;spacing:10
            RowLayout {Layout.fillWidth:true
                Text {text:"진입 금지 구역";color:"#E8EEF6";font.pixelSize:15;Layout.fillWidth:true}
                ActionButton {text:"닫기";compact:true;kind:"ghost";onClicked:keepoutTools.close()}
            }
            Text {text:"구역을 추가한 뒤 지도를 저장하면\n다시 열 때 함께 복원됩니다.";color:"#9AAFC8";font.pixelSize:11;Layout.fillWidth:true;wrapMode:Text.Wrap}
            ActionButton {objectName:"addKeepoutButton";text:page.ui.keepoutBusy?"구역 적용 중…":"사각형 구역 추가";kind:"primary";Layout.fillWidth:true;enabled:page.keepoutEditable&&Boolean(page.ui.mapReady);onClicked:page.beginKeepout()}
            Text {visible:!page.ui.keepoutAvailable;text:"금지 구역 서비스 연결을 기다리는 중입니다";color:"#D6AC78";font.pixelSize:10;wrapMode:Text.Wrap;Layout.fillWidth:true}
            Text {visible:page.keepouts.length===0;text:"설정된 구역이 없습니다";color:"#8BA4C0";font.pixelSize:12;Layout.fillHeight:true;verticalAlignment:Text.AlignVCenter}
            ListView {
                id:keepoutList;objectName:"keepoutList";visible:page.keepouts.length>0;model:page.keepouts;clip:true;Layout.fillWidth:true;Layout.fillHeight:true;spacing:4
                ScrollBar.vertical:ScrollBar{}
                delegate:Rectangle {
                    required property var modelData; required property int index
                    width:keepoutList.width;height:41;radius:5;color:"#2C343D"
                    RowLayout {anchors.fill:parent;anchors.margins:6
                        Rectangle {width:10;height:10;color:"#DA6C77";radius:2}
                        Text {text:"구역 "+(index+1)+" · "+(modelData.x_max-modelData.x_min).toFixed(1)+" × "+(modelData.y_max-modelData.y_min).toFixed(1)+" m";color:"#E4B8BE";font.pixelSize:11;Layout.fillWidth:true}
                        ActionButton {objectName:"deleteKeepout"+index;text:"삭제";compact:true;kind:"ghost";enabled:page.keepoutEditable;onClicked:operatorModel.removeKeepout(index)}
                    }
                }
            }
        }
    }
    Popup {
        id: mapPathDialog; objectName: "mapPathDialog"
        anchors.centerIn: parent; width: 560; height: 226; padding: 20; modal: true; focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle { color: "#F0192B3F"; radius: 10; border.color: "#6382A3" }
        ColumnLayout {
            anchors.fill: parent; spacing: 12
            Text { text: page.mapPathOperation==="save"?"현재 지도 저장":"저장 지도 열기"; color: "#E8EEF6"; font.pixelSize: 17 }
            Text { text: page.mapPathOperation==="save"?"저장할 지도 경로를 입력하세요. 확장자는 자동으로 붙습니다.":"저장된 지도의 .yaml 파일 경로를 입력하세요."; color: "#9AAAC0"; font.pixelSize: 12; wrapMode: Text.Wrap; Layout.fillWidth: true }
            Field { id: mapPath; objectName: "mapPathInput"; Layout.fillWidth: true; placeholderText: page.mapPathOperation==="save"?"/경로/지도이름":"/경로/지도이름.yaml"; selectByMouse: true }
            Item { Layout.fillHeight: true }
            RowLayout {
                Layout.alignment: Qt.AlignRight; spacing: 8
                ActionButton { objectName: "cancelMapPathButton"; text: "취소"; onClicked: mapPathDialog.close() }
                ActionButton { objectName: "mapPathSubmit"; text: page.mapPathOperation==="save"?"저장":"열기"; kind: "primary"; enabled: mapPath.text.trim()!==""; onClicked: { if(page.mapPathOperation==="save")operatorModel.saveMap(mapPath.text.trim());else operatorModel.loadMap(mapPath.text.trim());mapPathDialog.close() } }
            }
        }
    }
    Item {anchors.fill:parent;visible:page.cameraKind!=="";z:60
        MouseArea {anchors.fill:parent;onClicked:function(mouse){if(mouse.x<largeCamera.x||mouse.x>largeCamera.x+largeCamera.width||mouse.y<largeCamera.y||mouse.y>largeCamera.y+largeCamera.height)page.cameraKind=""}}
        Rectangle {id:largeCamera;objectName:"expandedCamera";anchors.centerIn:parent;width:Math.min(780,parent.width-80);height:Math.min(520,parent.height-60);radius:10;color:"#F0101B28";border.color:"#567492"
            ColumnLayout {anchors.fill:parent;anchors.margins:16;spacing:12
                RowLayout {Layout.fillWidth:true;Text {text:page.cameraKind==="depth"?"깊이 영상":"전방 RGB";color:"#E8EEF6";font.pixelSize:17;Layout.fillWidth:true}ActionButton {objectName:"closeCameraButton";text:"닫기";compact:true;onClicked:page.cameraKind=""}}
                CameraScene {Layout.fillWidth:true;Layout.fillHeight:true;depth:page.cameraKind==="depth"}
            }
        }
    }
}
