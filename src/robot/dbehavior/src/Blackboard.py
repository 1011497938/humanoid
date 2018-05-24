from dmsgs.msg import MotionInfo, VisionInfo, ActionCommand, BehaviorInfo, GCInfo, HeadCommand, TeamInfo
from dmsgs.srv import ToggleAMCL, ResetParticlePoint, ResetParticleLeftTouch, ResetParticleRightTouch
from Parameters import Parameters
from utils.actioncommand import crouch
from utils.VecPos import VecPos
from utils.mathutil import get_angle, get_magnitude, degree_between, Inf
from FieldGeometry import *
from Timer import Timer
from copy import copy
from math import pi, cos, sin
import rospy

class Blackboard(object):
    def __init__(self):
        self.motionInfo = MotionInfo()
        self.visionInfo = VisionInfo()
        self.actionCmd = ActionCommand()
        self.behaviorInfo = BehaviorInfo()
        self.behaviorInfo.time_to_reach_ball = Inf
        self.GCInfo = GCInfo()
        self.GCInfo.connected = False
        self.teamInfo = [None] * 6
        self.last_recv_motion_timestamp = rospy.Time(0)
        self.up_timestamp = rospy.Time.now()
        self.uptime = 0
        self.motionConnected = False
        self.see_ball_cnt = 0
        self.lost_ball_cnt = 0

        self.reentry = False
        self.role_init_pos = VecPos(0, 0, 0)

        # update info
        self.parameters = Parameters()
        robotId = self.parameters.robotId
        self.id = robotId
        rospy.Subscriber('/dmotion_{}/MotionInfo'.format(robotId), MotionInfo, self.updateMotionInfo)
        rospy.Subscriber('/dvision_{}/VisionInfo'.format(robotId), VisionInfo, self.updateVisionInfo)
        rospy.Subscriber('/dnetwork_{}/GCInfo'.format(robotId), GCInfo, self.updateGCInfo)
        rospy.Subscriber('/dnetwork_{}/TeamInfo'.format(robotId), TeamInfo, self.updateTeamInfo)
        self.cmdPub = rospy.Publisher('/dbehavior_{}/ActionCommand'.format(robotId), ActionCommand, queue_size=1)
        self.behaviorInfoPub = rospy.Publisher('/dbehavior_{}/BehaviorInfo'.format(robotId), BehaviorInfo, queue_size=1)

        # info
        self.oobot_pos = VecPos()
        self.robot_vision_inited = False
        self.see_ball = False
        self.ball_field = VecPos()
        self.last_seen_ball_field = VecPos(Inf, Inf)
        self.ball_global = VecPos()
        self.field_angle = 0
        self.vy = 0
        self.last_deltaData = VecPos()
        self.delta = VecPos()
        self.deltaZPre = 0
        self.deltaZ = 0
        self.robot_deltaZ = 0
        self.role = BehaviorInfo.Striker
        self.role = self.parameters.defaultRole
        self.t1 = Timer(self.parameters.motionInitTime)
        self.attackRight = self.parameters.attackRight
        self.last_closest_to_ball_timestamp = rospy.Time(0)
        self.robot_pos = VecPos()

        # timer
        self.ball_lost = Timer(10)

        # shared info
        self.attack_target = VecPos(550, 0)
        self.enable_kick = False

    def toggle_amcl(self):
        service_name = '/dvision_{}/toggle_amcl'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ToggleAMCL)
            func(True)
        except rospy.ServiceException, e:
            rospy.logerr('call toggle amcl error {}'.format(e))

    def reset_particle_left_touch(self):
        service_name = '/dvision_{}/reset_particles_left_touch'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticleLeftTouch)
            func()
        except rospy.ServiceException, e:
            rospy.logerr('call reset particle left touch error {}'.format(e))

    def reset_particle_right_touch(self):
        service_name = '/dvision_{}/reset_particles_right_touch'.format(self.id)
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, ResetParticleRightTouch)
            func()
        except rospy.ServiceException, e:
            rospy.logerr('call reset particles right touch error {}'.format(e))

    def reset_particle_point(self, point):
        pass
        # service_name = '/dvision_{}/reset_particles_point'.format(self.id)
        # rospy.wait_for_service(service_name)
        # try:
        #     func = rospy.ServiceProxy(service_name, ResetParticlePoint)
        #     func(point)
        # except rospy.ServiceException, e:
        #     rospy.logerr('call reset particles point error {}'.format(e))

    def resetCmd(self):
        # reset action command
        self.actionCmd = ActionCommand()
        self.actionCmd.bodyCmd = crouch()
        #$#####self.actionCmd.headCmd = HeadCommand(15, 0, 1, 1)

        # reset behaviour info
        #self.behaviorInfo = BehaviorInfo()
        self.behaviorInfo.save_image = False
        # self.behaviorInfo.current_role = None

    def checkMotionConnection(self):
        self.uptime = (rospy.Time.now() - self.up_timestamp).to_sec()
        elapsed = (rospy.Time.now() - self.last_recv_motion_timestamp).to_sec()
        if self.uptime > 1 and elapsed > 1:
            self.motionConnected = False
            self.reentry = True
            self.t1.restart()
        else:
            self.motionConnected = True

    def checkReentry(self):
        # 1. motion reconnect
        # 3. uptime < secs start
        pass

    def updateMotionInfo(self, msg):
        self.last_recv_motion_timestamp = rospy.Time.now()
        self.motionInfo = msg
        self.vy = msg.vy
        self.deltaZPre = self.last_deltaData.z
        self.deltaZ = self.last_deltaData.z - self.deltaZPre

        # Get delta
        tmpDelta = msg.deltaData
        dx = tmpDelta.x - self.last_deltaData.x
        dy = tmpDelta.y - self.last_deltaData.y
        dt = tmpDelta.z - self.last_deltaData.z

        t = self.last_deltaData.z / 180.0 * pi

        ddx = dx * cos(-t) - dy * sin(-t)
        ddy = dx * sin(-t) + dy * cos(-t)

        if self.last_deltaData.x != 0 or self.last_deltaData.y != 0 or self.last_deltaData.z != 0:
            self.delta.x = ddx
            self.delta.y = ddy
            self.delta.z = dt

        self.last_deltaData = tmpDelta

    def updateVisionInfo(self, msg):
        self.visionInfo = msg
        self.robot_pos = VecPos.fromVector3(msg.robot_pos)
        if not self.robot_vision_inited:
            self.robot_vision_inited = True
        else:
            self.robot_deltaZ = self.robot_pos.z - self.robot_pos_pre.z
            if self.robot_deltaZ > 180:
                self.robot_deltaZ -= 2 * 180
            elif self.robot_deltaZ < -180:
                self.robot_deltaZ += 2 * 180
        self.robot_pos_pre = self.robot_pos
        self.field_angle = self.robot_pos.z
        self.see_ball = self.visionInfo.see_ball
        self.ball_field = VecPos.fromVector3(msg.ball_field)
        self.ball_velocity = VecPos.fromVector3(msg.ball_velocity)
        self.ball_global = VecPos.fromVector3(msg.ball_global)

        if self.visionInfo.see_ball:
            self.last_seen_ball_field = self.ball_field.copy()
            self.ball_lost.restart()
            self.lost_ball_cnt = 0
            self.see_ball_cnt += 1
        else:
            self.see_ball_cnt = 0
            self.lost_ball_cnt += 1

        if False:
            print 'ball field: ', self.ball_field
            print 'ball global: ', self.ball_global
            print 'robot pos: ', self.robot_pos

    def updateGCInfo(self, msg):
       # print 'update GCInfo'
        self.GCInfo = msg

    def updateTeamInfo(self, msg):
        # print msg
        # print 'update team info', msg.player_number, msg.behavior_info.time_to_reach_ball
        self.teamInfo[msg.player_number - 1] = msg

    def getTeamInfo(self, id):
        info = self.teamInfo[id - 1]
        if info is None:
            return None

        now = rospy.Time.now()
        elapsed = now - info.recv_timestamp

        if elapsed.to_sec() > 3:
            #print id, 'timeout'
            return None
        elif info.incapacitated:
            # print id, 'incapacitated'
            return None

        return info

    def getTeamBallInfo(self):
        team_ball_global = []
        team_ball_confidence = []

        for id in [number for number in range(6) if number != self.id]:
            team_info = self.getTeamInfo(id)
            if team_info is not None and team_info.see_ball:
                team_ball_global.append(VecPos.fromVector3(team_info.ball_global))
                # TODO(corenel) refine the representation of confidence
                team_ball_confidence.append(1 / get_magnitude(VecPos.fromVector3(team_info.ball_field)))
        if len(team_ball_confidence) > 0:
            weighted_team_ball = VecPos()
            weighted_team_ball.x = sum(ball.x * score for ball, score in zip(team_ball_global, team_ball_confidence)) / sum(team_ball_confidence)
            weighted_team_ball.y = sum(ball.y * score for ball, score in zip(team_ball_global, team_ball_confidence)) / sum(team_ball_confidence)
            return weighted_team_ball
        else:
            return None

    def publish(self):
        self.calc_time()
        self.behaviorInfo.current_role = self.role
        self.cmdPub.publish(self.actionCmd)
        self.behaviorInfoPub.publish(self.behaviorInfo)

    def calc_time(self):
        # time to ball
        if (self.visionInfo.see_ball):
            rub = self.robot_pos - self.ball_global
            theta = degree_between(self.attack_target, self.ball_global)
            rub.rotate(-theta)

            angle = get_angle(rub)
            dis = get_magnitude(rub)
            self.behaviorInfo.time_to_reach_ball = abs(angle / self.parameters.turn_speed) + dis / self.parameters.walk_speed
            # if 0 < rub.x < 30 and -30 < rub.y < 30:
            #     self.behaviorInfo.time_to_reach_ball = 0
        else:
            self.behaviorInfo.time_to_reach_ball = Inf

        # time to goalie

        # time to striker

        # time to midfielder

    def teammates_has_striker(self):
        for id in xrange(1, 7):
            if id is self.id:
                pass
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    pass
                else:
                    role = t.behavior_info.current_role
                    if role is BehaviorInfo.Striker:
                        return True

        return False

    def teammates_has_defender(self):
        for id in xrange(1, 7):
            if id is self.id:
                pass
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    pass
                else:
                    role = t.behavior_info.current_role
                    if role is BehaviorInfo.Defender:
                        return True

        return False

    def teammates_has_midfielder(self):
        for id in xrange(1, 7):
            if id is self.id:
                pass
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    pass
                else:
                    role = t.behavior_info.current_role
                    if role is BehaviorInfo.MidFielder:
                        return True

        return False

    def teammates_see_ball(self):
        for id in xrange(1, 7):
            if id is self.id:
                pass
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    pass
                else:
                    if t.see_ball:
                        return True

        return False

    def team_lost_ball(self):

        for id in xrange(1, 7):
            if id is self.id:
                pass
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    pass
                else:
                    if t.see_ball:
                        return False
        return True

    def closest_to_ball(self):
        minTime = Inf
        minId = 0

        for id in xrange(1, 7):
            if id is self.id:
                time = self.behaviorInfo.time_to_reach_ball
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    time = Inf
                else:
                    time = t.behavior_info.time_to_reach_ball
                    # print id, time

            if time < minTime:
                minId = id
                minTime = time

        b = (minId == self.id)
        # print 'minId', minId, 'minTime', minTime, b

        if b:
            self.last_closest_to_ball_timestamp = rospy.Time.now()
            return True
        else:
            # elapsed = (rospy.Time.now() - self.last_closest_to_ball_timestamp).to_sec()
            #
            # if minTime != Inf:
            #     if elapsed > 5:
            #         return False
            #
            # if minTime == Inf:
            #     if elapsed > 15:
            #         return False

            return False

    def team_min_ball(self):
        minTime = Inf
        minId = -1

        for id in xrange(1, 7):
            if id is self.id:
                time = self.behaviorInfo.time_to_reach_ball
            else:
                t = self.getTeamInfo(id)
                if t is None:
                    time = Inf
                else:
                    time = t.behavior_info.time_to_reach_ball
                    # print id, time

            if time < minTime:
                minId = id
                minTime = time

        return minId, minTime


    def get_default_dest(self):
        if self.role == BehaviorInfo.Striker:
            if self.GCInfo.kickoff:
                dest = StrikerLeftPosKickoff
            else:
                dest = StrikerLeftPos
        elif self.role == BehaviorInfo.Defender:
            dest = DefenderLeftPos
        elif self.role == BehaviorInfo.MidFielder:
            dest = MidfielderLeftPos
        elif self.role == BehaviorInfo.Goalie:
            dest = GoalieLeftPos
        else:
            dest = StrikerLeftPos

        if not self.attackRight:
            dest = dest.mirror()

        return dest

"""
Blackboard singleton.
Global blackboard instance, everyone should use this same object, reading and writing this bb.
"""
_blackboard = Blackboard()


def attackRight():
    global _blackboard
    return _blackboard.parameters.attackRight

def getbb():
    """
    Get global blackboard instance.
    :return: Blackboard
    """
    global _blackboard
    return _blackboard
