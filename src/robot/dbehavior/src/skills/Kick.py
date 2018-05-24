from DecisionMaking.BehaviorTree.Task import Action, Task
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from DecisionMaking.BehaviorTree.Decorator import condition, seeBall, farFromObstacle
from skills.ApproachBall import ApproachBall
from skills.Lineup import Lineup
from Timer import Timer

#odometer
@seeBall
# @farFromObstacle
class _Kick(Action):
    def init(self):
        self.kick_timer = Timer()
        self.kicking = False

    def tick(self):
        # print '_Kick tick'
        if not self.kicking:
            self.kicking = True
            self.kick_timer.restart()

        if self.kick_timer.elapsed() < .5:
            self.lookAt(43, 0)
            self.crouch()
            return self.running()

        elif self.kick_timer.elapsed() < 2:
            self.lookAt(15, 0)

            closer_to_left_foot = self.bb.ball_field.y > 0
            can_left_kick = self.bb.parameters.left_kick
            can_right_kick = self.bb.parameters.right_kick

            # if self.bb.left_kick:
            #     self.kickLeft()
            # else:
            #     self.kickRight()

            if can_left_kick and closer_to_left_foot:
                self.kickLeft()
            elif can_right_kick and not closer_to_left_foot:
                self.kickRight()
            else:
                # print 'kick fail'
                return self.failure()

            # print 'kick running'
            return self.running()

        else:
            self.lookAt(15, 0)
            return self.success()

# class EnableKick(Task):
#     def init(self):
#         pass
#
#     def tick(self):
#         if self.bb.enable_kick:
#             return self.success()
#         else:
#             return self.failure()

@condition
def EnableKick(self):
    # print 'fuck'
    return self.bb.enable_kick


Kick = sequence(
                EnableKick,
                #ApproachBall,
                Lineup,
                _Kick)
