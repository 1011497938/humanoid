from DecisionMaking.BehaviorTree.Task import Action, Task
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from DecisionMaking.BehaviorTree.Decorator import condition, seeBall, farFromObstacle
from skills.ApproachBall import ApproachBall
from skills.Lineup import Lineup
from Timer import Timer

class Lie(Action):
    def init(self):
        self.lie_timer = Timer(5)
        self.inited = False

    def tick(self):
        if not self.inited:
            self.inited = True
            self.lie_timer.restart()
        if self.lie_timer.finished():
            print 'lie finished'
            return self.success()
        else:
            return self.running()
