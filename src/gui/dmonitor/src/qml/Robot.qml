import QtQuick 2.0
import DMonitor 1.0

Robot {
    id: robot
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        drag.target: robot
        onClicked: console.log("clicked on robot")
    }
}
