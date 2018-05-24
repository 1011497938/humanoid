from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.TrackBall import TrackBall
from utils.mathutil import *

DEST_REGION = 15
DEST_RE_ANGLE = 15
x = 450 * .8
y = 300 * .8


goals = [VecPos(x, y, 0),
         VecPos(x, -y, -90),
         VecPos(-x, -y, -180),
         VecPos(-x, y, 90)]

class _WalkAround(Action):
    def init(self):
        self.currentTarget = 0

    def tick(self):
        if self.got_dest(goals[self.currentTarget]):
            self.currentTarget += 1
            if self.currentTarget == 4:
                self.currentTarget = 0

        self.gotoGlobal(goals[self.currentTarget])


        # self.walk(1, 0, 3)
        # track = self.bb.visionInfo.ballTrack
        # self.lookAt(track.pitch, track.yaw)
        # self.gotoGlobal(VecPos(0, 0, 0))
        # self.gotoField(VecPos.fromVector3(self.bb.visionInfo.ball_field))
        #self.walksolver.solveGlobal(VecPos(0, 0, 0))


        return self.running()

    def got_dest(self, dest):
        robot_pos = self.bb.robot_pos
        angle = self.bb.field_angle
        dangle = abs_angle_diff(dest.z - angle)

        diff_x = dest.x - robot_pos.x
        diff_y = dest.y - robot_pos.y

        if not self.bb.enable_kick:
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
        elif self.bb.enable_kick:
            if abs(diff_x) < 5 and abs(diff_y) < 5 and abs(dangle) < 5:
                return True
            else:
                return False
        else:
            return False


WalkAround = parallel(_WalkAround, TrackBall)
