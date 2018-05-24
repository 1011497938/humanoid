from DecisionMaking.BehaviorTree.Task import Task
from dmsgs.msg import BehaviorInfo
from roles.Striker import Striker
from roles.Defender import Defender
# from roles.Goalie import Goalie
from roles.Midfielder import Midfielder
import time
import rospy

# TODO(mwx): delete this since we don't change rule
class Role(Task):
    """
    Set role, and tick
    """
    def init(self):
        self.striker = Striker()
        self.defender = Defender()
        self.midfielder = Midfielder()
        # self.goalie = Goalie()

        if self.bb.parameters.defaultRole == BehaviorInfo.Striker:
            self.defaultRole_instance = self.striker

        elif self.bb.parameters.defaultRole == BehaviorInfo.Defender:
            self.defaultRole_instance = self.defender

        elif self.bb.parameters.defaultRole == BehaviorInfo.MidFielder:
            self.defaultRole_instance = self.midfielder

        # elif self.bb.parameters.defaultRole == BehaviorInfo.Goalie:
        #     self.defaultRole_instance = self.goalie

        else:
            self.defaultRole_instance = self.striker

        self.currentRole_instance = self.defaultRole_instance
        self.prevRole_instance = self.currentRole_instance

        print '[python] Robot {} plays {}.'.format(self.bb.id, self.get_role_str(self.fuck(self.defaultRole_instance)))

    def fuck(self, role_instance):
        if role_instance == self.striker:
            return BehaviorInfo.Striker
        elif role_instance == self.defender:
            return BehaviorInfo.Defender
        elif role_instance == self.midfielder:
            return BehaviorInfo.MidFielder
        elif role_instance == self.goalie:
            return BehaviorInfo.Goalie
        else:
            return None

    def get_role_str(self, role):
        if role == BehaviorInfo.Striker:
            return "Striker"
        elif role == BehaviorInfo.Defender:
            return "Defender"
        elif role == BehaviorInfo.Goalie:
            return "Goalie"
        elif role == BehaviorInfo.MidFielder:
            return "Midfielder"
        else:
            return "Unkonwn"

    def tick(self):
        self.currentRole_instance.tick()
        return self.running()

