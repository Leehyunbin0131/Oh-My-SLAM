import QtQuick

Canvas {
    id: icon
    property string name: "map"
    property color ink: "#9AAAC0"
    implicitWidth: 20; implicitHeight: 20
    onNameChanged: requestPaint()
    onInkChanged: requestPaint()
    onPaint: {
        var c=getContext("2d"); c.reset(); c.scale(width/24,height/24);
        c.strokeStyle=ink; c.fillStyle=ink; c.lineWidth=1.65; c.lineCap="round"; c.lineJoin="round";
        function line(a,b,d,e){c.beginPath();c.moveTo(a,b);c.lineTo(d,e);c.stroke();}
        function box(x,y,w,h){c.strokeRect(x,y,w,h);}
        function ring(x,y,r){c.beginPath();c.arc(x,y,r,0,Math.PI*2);c.stroke();}
        if(name==="map") {c.beginPath();c.moveTo(3,5);c.lineTo(9,3);c.lineTo(15,5);c.lineTo(21,3);c.lineTo(21,19);c.lineTo(15,21);c.lineTo(9,19);c.lineTo(3,21);c.closePath();c.stroke();line(9,3,9,19);line(15,5,15,21);}
        else if(name==="camera") {box(3,7,18,13);box(8,4,8,3);ring(12,13.5,3.5);}
        else if(name==="motor") {box(6,7,12,10);box(2,9,4,6);box(18,9,4,6);line(12,3,12,7);line(12,17,12,21);ring(12,12,2.5);}
        else if(name==="system") {box(3,3,18,7);box(3,14,18,7);ring(7,6.5,.7);ring(7,17.5,.7);line(13,6.5,18,6.5);line(13,17.5,18,17.5);}
        else if(name==="target") {ring(12,12,7);ring(12,12,2);line(12,1,12,5);line(12,19,12,23);line(1,12,5,12);line(19,12,23,12);}
        else if(name==="layers") {c.beginPath();c.moveTo(3,8);c.lineTo(12,3);c.lineTo(21,8);c.lineTo(12,13);c.closePath();c.stroke();line(3,12,12,17);line(12,17,21,12);line(3,16,12,21);line(12,21,21,16);}
        else if(name==="play") {c.beginPath();c.moveTo(7,4);c.lineTo(20,12);c.lineTo(7,20);c.closePath();c.stroke();}
        else if(name==="stop") {box(5,5,14,14);}
        else if(name==="plus") {line(5,12,19,12);line(12,5,12,19);}
        else if(name==="minus") {line(5,12,19,12);}
        else if(name==="chevron") {line(9,5,16,12);line(16,12,9,19);}
        else if(name==="expand") {line(4,10,4,4);line(4,4,10,4);line(14,4,20,4);line(20,4,20,10);line(4,14,4,20);line(4,20,10,20);line(14,20,20,20);line(20,20,20,14);}
        else if(name==="folder") {c.beginPath();c.moveTo(3,6);c.lineTo(10,6);c.lineTo(12,9);c.lineTo(21,9);c.lineTo(21,20);c.lineTo(3,20);c.closePath();c.stroke();}
        else if(name==="robot") {box(5,6,14,14);ring(9,12,1);ring(15,12,1);line(9,16,15,16);line(12,2,12,6);line(2,10,2,16);line(22,10,22,16);}
        else if(name==="clock") {ring(12,12,9);line(12,6,12,12);line(12,12,16,15);}
        else if(name==="check") {line(5,12,10,17);line(10,17,20,6);}
        else if(name==="close") {line(6,6,18,18);line(18,6,6,18);}
        else if(name==="info") {ring(12,12,9);line(12,11,12,17);ring(12,7,.6);}
        else if(name==="arrow") {line(4,12,20,12);line(14,6,20,12);line(20,12,14,18);}
    }
}
