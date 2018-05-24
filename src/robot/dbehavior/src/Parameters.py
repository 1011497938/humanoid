import rospy
from dmsgs.msg import BehaviorInfo
from utils.VecPos import VecPos

class Parameters(object):
    """
    All rosparams.
    """
    def __init__(self):
        self.robotId = rospy.get_param('~RobotId')

        # self.robotId = 1
        self.skill = rospy.get_param('~skill')
        self.motionInitTime = rospy.get_param('/dmotion_{}/dmotion/hardware/imu_prepare_time'.format(self.robotId))
        self.attackRight = rospy.get_param('/ZJUDancer/AttackRight')

        roleid = rospy.get_param('~dbehavior/role')
        if roleid == "Striker":
            self.defaultRole = BehaviorInfo.Striker
        elif roleid == "Defender":
            self.defaultRole = BehaviorInfo.Defender
        elif roleid == "Mid":
            self.defaultRole = BehaviorInfo.MidFielder
        else:
            self.defaultRole = BehaviorInfo.Goalie

        self.enableLog = False

        # Constants
        self.maxPitch = 45
        self.minPitch = 0
        self.maxYaw = 120
        self.walk_speed = 450 / 26
        self.turn_speed = 10

        self.GoaliePointX = 420

        # kick
        if self.robotId is 1:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        elif self.robotId is 2:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        elif self.robotId is 3:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        elif self.robotId is 4:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        elif self.robotId is 5:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        elif self.robotId is 6:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)

        else:
            self.left_kick = True
            self.right_kick = True
            self.left_kick_point = VecPos(15, 4)
            self.right_kick_point = VecPos(15, -4)
