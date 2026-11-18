import QtQuick
import QtQuick.Layouts

Item {
    id:plot
    property var series: []
    property var lineColors: ["#6AA9FF","#63CFB1","#DDB56B"]
    property string title: ""
    property string unit: ""
    property real minimum:-1
    property real maximum:1
    implicitHeight:180
    onSeriesChanged: graph.requestPaint()
    onMinimumChanged: graph.requestPaint()
    onMaximumChanged: graph.requestPaint()
    Text { id:label; text:plot.title; color:"#BECBDC"; font.pixelSize:12; anchors.top:parent.top }
    Text { text:plot.unit; color:"#8294AC"; font.pixelSize:11; anchors.right:parent.right }
    Canvas {
        id:graph; anchors.fill:parent; anchors.topMargin:26; anchors.bottomMargin:20; anchors.leftMargin:29
        onPaint: {
            var c=getContext("2d");c.reset();c.lineWidth=1;
            for(var i=0;i<4;i++){var y=3+i*(height-6)/3;c.strokeStyle="#293546";c.beginPath();c.moveTo(0,y);c.lineTo(width,y);c.stroke();}
            var data=plot.series;
            if(!data.length){data=[];for(var k=0;k<2;k++){var arr=[];for(var n=0;n<70;n++)arr.push(.15*Math.sin(n*.24+k)+.08*Math.cos(n*.8)+.12*k);data.push(arr);}}
            for(var s=0;s<data.length;s++) {c.strokeStyle=plot.lineColors[s%plot.lineColors.length];c.lineWidth=1.7;c.beginPath();for(var p=0;p<data[s].length;p++){var val=typeof data[s][p]==="number"?data[s][p]:data[s][p][1];var px=p*width/Math.max(1,data[s].length-1);var py=height-(val-plot.minimum)/(plot.maximum-plot.minimum)*height;if(p===0)c.moveTo(px,py);else c.lineTo(px,py);}c.stroke();}
        }
    }
    Text { text:plot.maximum.toFixed(1); color:"#73859C"; font.pixelSize:10; y:25 }
    Text { text:plot.minimum.toFixed(1); color:"#73859C"; font.pixelSize:10; anchors.bottom:parent.bottom; anchors.bottomMargin:19 }
    RowLayout { anchors.left:parent.left; anchors.leftMargin:29; anchors.right:parent.right; anchors.bottom:parent.bottom
        Text {text:"−30 s";color:"#73859C";font.pixelSize:10}
        Item {Layout.fillWidth:true}
        Text {text:"−15 s";color:"#73859C";font.pixelSize:10}
        Item {Layout.fillWidth:true}
        Text {text:"0 s";color:"#73859C";font.pixelSize:10}
    }
}
