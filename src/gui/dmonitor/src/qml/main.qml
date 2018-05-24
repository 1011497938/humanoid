import QtQuick 2.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Controls 1.2
import DMonitor 1.0
//import "../js/componentCreation.js" as Spawner

ApplicationWindow {
    id: root
    visible: true
    width: drawArea.width + guiImage.width + 10 * 3
    height: drawArea.height + infoArea.height +  10 * 3
    title: qsTr("DMonitor")
    color: "black"
    //color: "#1B2B34"

    property string udpAddress: '127.0.0.1'
    property string borderColor: "red"

//    Rectangle {
//        id: drawArea
//        x: 10
//        y: 10
//        height: field.height
//        width: height * 1.618
//        color: "black"
//       // color: "#1B2B34"

//        signal selectRobot(int id)

//        property var robots: []
//        property var balls: []
//        property alias field: field
//        property bool isMonitor: true

//        Timer {
//            interval: 1000 / 20
//            running: true
//            repeat: true
//            onTriggered: drawArea.updateView()
//        }

//        // create field before robots
//        Field {
//            id: field
//            width: 1040 / 1.5;
//            height: 740 / 1.5;
//            anchors.left: parent.left
//            MouseArea {
//                anchors.fill: parent
//                onClicked: console.log('click field')
//            }
//        }


//        Box {
//            id: selection
//            width: parent.width - field.width - 10
//            height: field.height
//            anchors.left: field.right
//            anchors.leftMargin: 10
//            anchors.top: parent.top
//            property var robots: drawArea.robots

//            Button {
//               x: 10
//               y: 10
//               id: modeSelection
//               width: selection.width - 20
//               property bool isMonitor: true
//               text: isMonitor ? "Monitor" : "Simulate"
//               onClicked: {
//                    isMonitor = !isMonitor;
//                    drawArea.isMonitor = isMonitor

//                    if(!isMonitor) {
//                        for(var i = 0; i < drawArea.robots.length; ++i) {
//                           drawArea.robots[i].reset()
//                        }
//                    }
//               }
//            }

//            // FIXME(MWX): !? seen when robots online
//            // reeating myself
//            Column {
//                x: 10
//                y: 100
//                spacing: 10
//                id: robotSelect
//                property int currentIndex: 1
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 1"
//                    visible: drawArea.robots[0].online
//                    onClicked: drawArea.selectRobot(1)
//                }
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 2"
//                    visible: drawArea.robots[1].online
//                    onClicked: drawArea.selectRobot(2)
//                }
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 3"
//                    visible: drawArea.robots[2].online
//                    onClicked: drawArea.selectRobot(3)
//                }
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 4"
//                    visible: drawArea.robots[3].online
//                    onClicked: drawArea.selectRobot(4)
//                }
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 5"
//                    visible: drawArea.robots[4].online
//                    onClicked: drawArea.selectRobot(5)
//                }
//                Button {
//                    width: selection.width - 20
//                    text: "Robot 6"
//                    visible: drawArea.robots[5].online
//                    onClicked: drawArea.selectRobot(6)
//                }
//            }
//        }

//        function updateView() {
//            robots.forEach(function(rbt, index, array) {
//                rbt.setIsMonitor(modeSelection.isMonitor);
//                rbt.update();
//                rbt.viewRange.update();
//            })

//            balls.forEach(function(ball, index, array){
//                ball.setIsMonitor(modeSelection.isMonitor);
//                ball.update();
//            })
//        }

//        Component.onCompleted: {
//            robots = Spawner.createRobots();
//            balls = Spawner.createBalls();
//            field.update();
//            drawArea.selectRobot.connect(gui.onSelectRobot)
//        }
//    }

//    Box {
//        id: guiImage
//        height: drawArea.height
//        width: height / (480 / 640)
//        anchors.left: drawArea.right
//        anchors.top: drawArea.top
//        anchors.leftMargin: 10

//        GuiImage {
//            id: gui
//            field: drawArea.field
//            robotId: 1
//            anchors.fill: parent
//            function onSelectRobot(id) {
//               console.log("select" ,id)
//                setCurrentId(id)
//            }

//            Timer {
//                interval: 1000 / 30
//                running: true
//                repeat: true
//                onTriggered:  {
//                    if(drawArea.isMonitor) {
//                        parent.update()
//                    }
//                }
//            }

//            Component.onCompleted: {
//                init();
//            }
//        }
//    }

//    Box {
//        id: infoArea
//        width: root.width - 20
////        height: drawArea.height
//        height: 0
//        anchors.left: drawArea.left
//        anchors.top: drawArea.bottom
//        anchors.topMargin: 10
//    }

    Window {
        visible: true
        title: "Map"
        width: 1040 + 5
        height: 740 + 5

        Rectangle {
            anchors.fill: parent
            Map {
                id: map
                anchors.fill: parent
            }
        }
    }

}
