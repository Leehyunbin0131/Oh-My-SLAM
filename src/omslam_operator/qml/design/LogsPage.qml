import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id:page
    property var events: []
    property string expandedMessage: ""
    signal notify(string message)
    ListModel { id: logRows }
    function syncEvents() {
        if (!logRows) return
        // ROS refreshes frequently; unchanged rows must retain scroll and focus.
        var changed=logRows.count!==events.length
        if(!changed)for(var i=0;i<events.length;i++){
            var old=logRows.get(i), next=events[i]
            if(old.time!==next.time||old.message!==next.message||old.count!==next.count){changed=true;break}
        }
        if(!changed)return
        var topIndex = logList && !logList.atYBeginning ? logList.indexAt(1,logList.contentY+1) : -1
        var top = topIndex >= 0 ? logRows.get(topIndex) : null
        var anchor = top ? [top.time,top.message] : null
        var item = anchor ? logList.itemAtIndex(topIndex) : null
        var offset = item ? logList.contentY-item.y : 0
        for(var j=0;j<events.length;j++) {
            var entry=events[j]
            var row={time:entry.time||"",level:entry.level||"정보",message:entry.message||"",count:entry.count||1}
            if(j<logRows.count && logRows.get(j).time===row.time && logRows.get(j).message===row.message)
                logRows.set(j,row)
            else
                logRows.insert(j,row)
        }
        if(logRows.count>events.length)logRows.remove(events.length,logRows.count-events.length)
        if(logList) {
            logList.forceLayout()
            if(!anchor) logList.positionViewAtBeginning()
            else for(var k=0;k<logRows.count;k++) {
                if(logRows.get(k).message===anchor[1] && logRows.get(k).time===anchor[0]) {
                    logList.positionViewAtIndex(k,ListView.Beginning)
                    logList.contentY+=offset
                    break
                }
            }
        }
    }
    onEventsChanged: syncEvents()
    Component.onCompleted: syncEvents()
    ColumnLayout {anchors.fill:parent;anchors.margins:28;spacing:18
        RowLayout {Layout.fillWidth:true;spacing:16
            Text {text:"시스템 로그";color:"#E8EEF6";font.pixelSize:23;font.weight:Font.DemiBold}
            Text {text:"실제 조작 기록 · 센서 상태 · 내용을 눌러 펼치기";color:"#9AAAC0";font.pixelSize:12}
            Item {Layout.fillWidth:true}
            Field {id:search;objectName:"logSearch";placeholderText:"로그 검색 · PWM, IMU 등";Layout.preferredWidth:260;Accessible.name:"로그 검색"}
        }
        Rectangle {Layout.fillWidth:true;Layout.fillHeight:true;color:"#151E2A";radius:10;border.color:"#304257"
            ColumnLayout {anchors.fill:parent;anchors.margins:20;spacing:0
                RowLayout {Layout.fillWidth:true;Layout.preferredHeight:36;spacing:20
                    Text {text:"시각";color:"#849BB7";Layout.preferredWidth:100}
                    Text {text:"구분";color:"#849BB7";Layout.preferredWidth:65}
                    Text {text:"내용";color:"#849BB7";Layout.fillWidth:true}
                }
                Rectangle {height:1;Layout.fillWidth:true;color:"#33475F"}
                Text {visible:page.events.length===0;text:"아직 수신된 로그가 없습니다.";color:"#8FA8C4";Layout.alignment:Qt.AlignHCenter;Layout.topMargin:24}
                ListView {id:logList;objectName:"logList";Layout.fillWidth:true;Layout.fillHeight:true;clip:true;model:logRows;spacing:0
                    delegate:Item {
                        objectName:"logRow"
                        required property string time;required property string level;required property string message;required property int count
                        width:ListView.view.width;height:matches?Math.max(52,messageText.contentHeight+24):0;visible:matches
                        property bool expanded: page.expandedMessage===message
                        property bool matches:search.text===""||(message+level+time).toLowerCase().indexOf(search.text.toLowerCase())>=0
                        TapHandler { onTapped: page.expandedMessage=parent.expanded?"":parent.message }
                        Rectangle {anchors.bottom:parent.bottom;width:parent.width;height:1;color:"#29384B"}
                        RowLayout {anchors.fill:parent;anchors.topMargin:12;anchors.bottomMargin:12;spacing:20
                            Text {text:time;color:"#9AAAC0";font.pixelSize:12;Layout.preferredWidth:100;Layout.alignment:Qt.AlignTop}
                            Text {text:level;color:level==="주의"?"#DDB56B":level==="상세"?"#9AAAC0":"#63CFB1";font.pixelSize:12;Layout.preferredWidth:65;Layout.alignment:Qt.AlignTop}
                            Text {id:messageText;objectName:"logMessage";text:message+(count>1?"  ×"+count:"");color:"#DFE9F5";font.pixelSize:13;Layout.fillWidth:true;Layout.alignment:Qt.AlignTop;wrapMode:Text.Wrap;maximumLineCount:parent.parent.expanded?2147483647:3;elide:parent.parent.expanded?Text.ElideNone:Text.ElideRight}
                        }
                    }
                    ScrollBar.vertical:ScrollBar{}
                }
            }
        }
    }
}
