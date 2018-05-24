import QtQuick 2.0
import DMonitor 1.0

Ball {
    id: ball
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        drag.target: ball
        onClicked: console.log("click")
    }
}

