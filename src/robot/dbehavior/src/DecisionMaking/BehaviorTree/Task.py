# TODO(MWX):
# onEnter and onLeave logic

from enum import Enum
from utils.VecPos import VecPos
from inspect import isclass
from math import fabs
from Blackboard import getbb
from utils.actioncommand import *
from utils.WalkToPointSolver import WalkToPointSolver
from utils.mathutil import *
from math import fabs, pi

# DEST_REGION = 15
# DEST_RE_ANGLE = 10

DEST_REGION = 25
DEST_RE_ANGLE = 15

class Status(Enum):
    """
    Wrap up all status we need.
    Fresh: the task has never run or has been reset
    Running: the task has not completed and needs to run again
    Failure: the task returned a failure result
    Success: the task returned a success result
    """
    FRESH = 0
    RUNNING = 1
    FAILURE = 2
    SUCCESS = 3

class Task(object):
    """
    Behavior tree Node
    The core class for DecisionMaking.
    """
    def __init__(self):
        # Status Code
        self.status = Status.FRESH
        self._children = []
        self.bb = getbb()
        self.init()


    def init(self):
        """
        This funciton is by default called after Task object initialised.
        You should not define __init__ function is subcalss, if you want
        to add custom variables, override this funciton.
        """
        pass

    def fresh(self):
        self.init()
        self.status = Status.FRESH
        for child in self._children:
            child.fresh()

    def isFresh(self):
        return self.status is Status.FRESH

    def onEnter(self):
        """
        Called on enter this task
        """
        pass

    def onLeave(self):
        """
        Called on leaeve this task
        """
        pass

    def success(self):
        self.status = Status.SUCCESS
        self.onLeave()
        return self.status

    # FIXME(MWX): status code needs more clear definition
    def failure(self):
        self.status = Status.FAILURE
        self.onLeave()
        return self.status

    def running(self):
        self.status = Status.RUNNING
        return self.status

    def addChild(self, task):
        """
        Add children to this node
        """
        if task is None:
            raise TypeError('Trying to add None as child')
        elif task is self:
            raise TypeError('Trying to add self as child')
        elif not isinstance(task, Task) and (isclass(task) and not issubclass(task, Task)):
            raise TypeError('Not an instance or subclass of Node')


        if isclass(task):
            self._children.append(task())
        else:
            self._children.append(task)

        return self

    def tick(self):
        raise NotImplementedError

class Action(Task):
    """
    Behavior tree leaf, who can change ActionCommand
    """
    def __init__(self):
        super(Action, self).__init__()
        self.walksolver = WalkToPointSolver()

    def init(self):
        pass

    def addChild(self, task):
        raise Exception('Actions are leaf node, must not have child')

    def do(self, cmd):
        self.bb.actionCmd.bodyCmd = cmd

    def walk(self, forward=0, left=0, turn=0):
        self.bb.actionCmd.bodyCmd = walk(forward, left, turn)

    def crouch(self):
        self.bb.actionCmd.bodyCmd = crouch()

    def stand(self):
        self.bb.actionCmd.bodyCmd = standup()

    def step(self):
        self.bb.actionCmd.bodyCmd = walk(0, 0, 0)

    def turn(self, st):
        self.bb.actionCmd.bodyCmd = walk(0, 0, st)

    def lookAt(self, pitch=0, yaw=0, pitchSpeed=1, yawSpeed=1):
        self.bb.actionCmd.headCmd = head(pitch, yaw, pitchSpeed, yawSpeed)

    def kickLeft(self):
        self.bb.actionCmd.bodyCmd = kickLeft()

    def kickRight(self):
        self.bb.actionCmd.bodyCmd = kickRight()

    def goalieLeft(self):
        self.bb.actionCmd.bodyCmd = goalieLeft()

    def goalieMid(self):
        self.bb.actionCmd.bodyCmd = goalieMid()

    def goalieRight(self):
        self.bb.actionCmd.bodyCmd = goalieRight()

    def capture(self):
        self.bb.behaviorInfo.save_image = True

    def gotoField(self, destField):
        x, y, t = self.walksolver.solveField(destField)
        self.walk(x, y, t)

    def gotoGlobal(self, dest):
        if self.got_dest_loose(dest):
            return True

        x, y, t = self.walksolver.solveGlobal(dest)
        self.bb.behaviorInfo.dest = dest
        self.walk(x, y, t)
        return False

    def gotoGlobalOmni(self, dest):
        if self.got_dest_loose(dest):
            return True

        x, y, t = self.walksolver.solveGlobal(dest, True)
        self.bb.behaviorInfo.dest = dest
        self.walk(x, y, t)
        return False

    def faceBall(self):
        if self.bb.see_ball:
            thresh = 20
            ball = self.bb.ball_field
            angle = atan2(ball.y, ball.x)

            if angle > radians(thresh):
                self.walk(0, 0, 10)
            elif angle < radians(-thresh):
                self.walk(0, 0, -10)
            else:
                self.crouch()

            self.gazeBall()

    def gazeBall(self):
        if self.bb.visionInfo.see_ball:
            track = self.bb.visionInfo.ballTrack
            curplat = self.bb.motionInfo.curPlat
            thresh = 1
            pitch = fabs(track.pitch - curplat.x) > thresh and track.pitch or curplat.x
            yaw = fabs(track.yaw - curplat.y) > thresh and track.yaw or curplat.y

            self.lookAt(pitch, yaw, 1, 1)
            return True
        else:
            return False

    def headGotDest(self, pitch, yaw):
        thresh = 1
        curplat = self.bb.motionInfo.curPlat
        return fabs(pitch - curplat.x) < thresh and fabs(yaw - curplat.y) < thresh

    def got_dest(self, dest):
        robot_pos = self.bb.robot_pos
        angle = self.bb.field_angle
        dangle = abs_angle_diff(dest.z - angle)

        diff_x = dest.x - robot_pos.x
        diff_y = dest.y - robot_pos.y

        if self.bb.ball_field.y > 0:
            if abs(diff_x) < DEST_REGION and -DEST_REGION < diff_y < DEST_REGION / 3 and abs(dangle) < DEST_RE_ANGLE:
                return True
            else:
                return False
        else:
            if abs(diff_x) < DEST_REGION and -DEST_REGION / 3 < diff_y < DEST_REGION and abs(dangle) < DEST_RE_ANGLE:
                return True
            else:
                return False

    def got_dest_loose(self, dest):
        robot_pos = self.bb.robot_pos
        angle = self.bb.field_angle
        dangle = abs_angle_diff(dest.z - angle)

        diff_x = dest.x - robot_pos.x
        diff_y = dest.y - robot_pos.y

        if self.bb.ball_field.y > 0:
            if abs(diff_x) < DEST_REGION * 1.5 and -DEST_REGION * 1.5 < diff_y < DEST_REGION * 1.5 / 3 and abs(dangle) < DEST_RE_ANGLE * 1.3:
                return True
            else:
                return False
        else:
            if abs(diff_x) < DEST_REGION * 1.5 and -DEST_REGION * 1.5 / 3 < diff_y < DEST_REGION * 1.5 and abs(dangle) < DEST_RE_ANGLE * 1.3:
                return True
            else:
                return False

    def got_dest_tight(self, dest):
        robot_pos = self.bb.robot_pos
        angle = self.bb.field_angle
        dangle = abs_angle_diff(dest.z - angle)

        diff_x = dest.x - robot_pos.x
        diff_y = dest.y - robot_pos.y

        if True:
            if self.bb.ball_field.y > 0:
                if abs(diff_x) < DEST_REGION and -DEST_REGION < diff_y < DEST_REGION / 3 and abs(dangle) < DEST_RE_ANGLE / 1.5:
                    return True
                else:
                    return False
            else:
                if abs(diff_x) < DEST_REGION and -DEST_REGION / 3 < diff_y < DEST_REGION and abs(dangle) < DEST_RE_ANGLE / 1.5:
                    return True
                else:
                    return False
        # elif self.bb.enable_kick:
        #     if abs(diff_x) < 5 and abs(diff_y) < 5 and abs(dangle) < 5:
        #         return True
        #     else:
        #         return False
        else:
            return False
