var Robot;
var Ball;
var ViewRange;
var robots = [];
var robotnum = 6;
var balls = [];

function createRobots() {
    // create robot
    Robot = Qt.createComponent("../qml/Robot.qml");
    ViewRange = Qt.createComponent("../qml/ViewRange.qml");


    if(Robot.status === Component.Ready)
        finishRobotCreation();
    else
        Robot.statusChanged.connect(finishCreation);


    return robots;
}

function createBalls() {
    // create ball
    Ball = Qt.createComponent("../qml/Ball.qml");

    if(Ball.status === Component.Ready)
        finishBallCreation();
    else
        Ball.statusChanged.connect(finishBallCreation);

    return balls;
}


function finishRobotCreation() {
   if(Robot.status === Component.Ready) {
       for(var i = 1; i <= robotnum; ++i) {
           var viewRange = ViewRange.createObject(drawArea, {
                                                   "width": 10,
                                                   "height": 10,
                                                    "field": field
                                                  });
           var rbt = Robot.createObject(drawArea, {"robotId": i,
                                                   "width": 10,
                                                   "height": 10,
                                                   "address": udpAddress,
                                                   "viewRange": viewRange,
                                                   "field": field})
           robots.push(rbt);
       }

       for(i = 0; i < robots.length; ++i) {
           robots[i].init();
       }
   } else if (Robot.status === Component.Error) {
       console.log('Error loading Robot: ', Robot.errorString())
   }
}


function finishBallCreation() {
   if(Ball.status === Component.Ready) {
       for(var i = 1; i <= robotnum + 1; ++i) {
           var bl = Ball.createObject(drawArea, {//"robotId": i,
                                                   "width": 10,
                                                   "height": 10,
                                                   //"address": udpAddress,
                                                   "field": field});
           balls.push(bl);
       }

       for(i = 0; i < robots.length; ++i) {
           balls[i].init();
           robots[i].ball = balls[i];
           robots[i].simBall = balls[robotnum];
       }
   } else if (Robot.status === Component.Error) {
       console.log('Error loading Robot: ', Robot.errorString())
   }
}
