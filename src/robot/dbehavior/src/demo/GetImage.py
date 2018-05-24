from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import sequence
from DecisionMaking.BehaviorTree.Decorator import repeatSec
from Parameters import Parameters
from Timer import Timer

param = Parameters()
diff_yaw = [i for i in range(-param.maxYaw, param.maxYaw, 20)]
diff_pitch = [i for i in range(param.minPitch, param.maxPitch, 10)]
path = []


for i in range(len(diff_pitch)):
    tmp = [diff_pitch[i] for j in range(len(diff_yaw))]
    path += zip(diff_yaw, tmp)
    diff_yaw.reverse()

# Move(1s) --> Keep(1s) --> Capture(1s) --> Move(1s)

currentPoint = (0, 0)
iteration = iter(path)

# wait
timer = Timer(5)
while not timer.finished():
    timer.sleep(1)


@repeatSec(0.5)
class Move(Action):
    def tick(self):
        self.lookAt(pitch=currentPoint[1], yaw=currentPoint[0])
        return self.running()

@repeatSec(0.5)
class Keep(Action):
    def tick(self):
        self.lookAt(pitch=currentPoint[1], yaw=currentPoint[0])
        return self.running()


@repeatSec(0.5)
class Capture(Action):
    def init(self):
        global currentPoint
        global iteration
        try:
            currentPoint = iteration.next()
        except StopIteration:
            iteration = iter(path)

    def tick(self):
        # TODO(MWX), Task need onEnter and onLeave
        print 'Cheese! ({}, {})'.format(currentPoint[1], currentPoint[0])
        self.bb.behaviorInfo.save_image = True
        return self.running()


GetImage = sequence(
    Move, Keep, Capture
)
