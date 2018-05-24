from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from skills.Kick import Kick

class _Foo(Action):
    def tick(self):
        self.bb.enable_kick = True
        print self.bb.ball_field
        return self.running()

TestKick = parallel(_Foo, Kick)
