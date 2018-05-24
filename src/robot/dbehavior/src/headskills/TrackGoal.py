from DecisionMaking.BehaviorTree.Task import Action
from math import fabs

class TrackGoal(Action):
    def tick(self):
        track = self.bb.visionInfo.ballTrack
        curplat = self.bb.motionInfo.curPlat
        curYaw = curplat.y

        thresh = 5
        pitch = fabs(track.pitch - curplat.x) > thresh and track.pitch or curplat.x
        yaw = fabs(track.yaw - curplat.y) > thresh and track.yaw or curplat.y

        self.lookAt(pitch, yaw, 1, 1)

        #self.lookAt(pitch= 15, yaw=0)
        # print "lootAt ({}, {})".format(track.pitch, track.yaw)
        # print self.bb.visionInfo.ball_field
        # print "lootAt ({}, {})".format(track.pitch, track.yaw)
        return self.success()
