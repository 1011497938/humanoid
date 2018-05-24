from DecisionMaking.BehaviorTree.Task import Action
from geometry_msgs.msg import Vector3
from Timer import Timer

class Init(Action):
    def tick(self):
        if self.bb.parameters.attackRight:
            self.bb.reset_particle_left_touch()
        else:
            self.bb.reset_particle_right_touch()

        self.lookAt(pitch=0, yaw=0)
        self.crouch()
        return self.running()
