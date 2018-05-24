from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Decorator import condition
from headskills.ScanField_fast import scanField_fast
from headskills.ScanField import scanField
from roles.Striker import Striker
from roles.Role import Role
from Timer import Timer
from utils.VecPos import VecPos

"""
Kick off right
"""

class Initial(Action):
    def init(self):
        self.timer = Timer(10)

    def onEnter(self):
        self.timer.restart()

    def tick(self):
        if self.bb.parameters.attackRight:
            self.bb.reset_particle_left_touch()
        else:
            self.bb.reset_particle_right_touch()

        self.lookAt(pitch=0, yaw=0)
        self.crouch()
        return self.running()

class Set(Action):
    """
    Face enemy goal
    """
    def tick(self):
        scanField.tick()
        self.crouch()

class Playing(Action):
    def init(self):
        self.timer = Timer(5)
        self.role = Role()
        self.seen_ball = False

    def tick(self):
        return self.role.tick()


class Finished(Action):

    def tick(self):
        self.stand()
        return self.success()
