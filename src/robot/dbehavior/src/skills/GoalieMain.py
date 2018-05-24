from skills.GoalieAttack import GoalieAttack
from skills.GoToGoaliePoint import GoToGoaliePoint
from skills.GoalieBack import GoalieBack
from DecisionMaking.BehaviorTree.Task import Task, Status
from DecisionMaking.BehaviorTree.Decorator import repeatSec
SafeDist = 200

class GoalieMain(Task):
    def init(self):
        self.cur_action = None
        self.cur_state = None
        self.last_state = None
        self.goalie_back = GoalieBack()
        self.goalie_attack = GoalieAttack()

    def tick(self):

        if (self.bb.attackRight and self.bb.ball_global.x < -SafeDist) or (not self.bb.attackRight and self.bb.ball_global.x > SafeDist):
            # is safe
            self.cur_action = self.goalie_attack

        else:
            # if self.cur_action is self.goalie_attack and self.cur_state is Status.SUCCESS and abs(self.bb.ball_global.x) < SafeDist:

            if self.cur_action is self.goalie_back and self.cur_state is Status.SUCCESS:
                return self.success()
            else:
                self.cur_action = self.goalie_back
            # return self.failure()
        # if self.last_state != self.cur_state:
        #     self.cur_state.onEnter()
        self.cur_state = self.cur_action.tick()
        return self.running()


        # self.last_state = self.cur_state
