from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Decorator import seeBall
from headskills.ScanField_fast import scanField_fast
from math import fabs

class TrackBall(Action):
    def tick(self):
        if not self.bb.see_ball:
            scanField_fast.tick()
            return self.running()
        else:
            ball_field = self.bb.ball_field
            if ball_field.x < 40 and fabs(ball_field.y) < 30:
                self.lookAt(45, 0)
            else:
                track = self.bb.visionInfo.ballTrack
                curplat = self.bb.motionInfo.curPlat
                curYaw = curplat.y

                thresh = 3
                pitch = fabs(track.pitch - curplat.x) > thresh and track.pitch or curplat.x
                yaw = fabs(track.yaw - curplat.y) > thresh and track.yaw or curplat.y

                self.lookAt(pitch, yaw, 1, 1)

                #self.lookAt(pitch= 15, yaw=0)
                # print "lootAt ({}, {})".format(track.pitch, track.yaw)
                # print self.bb.visionInfo.ball_field
                # print "lootAt ({}, {})".format(track.pitch, track.yaw)
        return self.success()

trackBall = TrackBall()