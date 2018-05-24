from skills.Reentry import Reentry
from states.GCPlay import GCPlay
from states.GCStates import Initial
from DecisionMaking.BehaviorTree.Task import Task
from DecisionMaking.BehaviorTree.Decorator import repeatSec


class Game(Task):
    def init(self):
        self.cur_state = None
        self.last_state = None
        self.gc_play = GCPlay()
        self.reentry = Reentry()

    def tick(self):
        if self.bb.reentry:
            self.cur_state = self.reentry
        else:
            self.cur_state = self.gc_play

        if self.last_state != self.cur_state:
            self.cur_state.onEnter()

        self.cur_state.tick()

        self.last_state = self.cur_state

        return self.running()
