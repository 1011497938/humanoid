from DecisionMaking.BehaviorTree.Task import Action
from math import fabs
from utils.VecPos import calc_field_position

class TrackCircle(Action):
    def tick(self):
        track = self.bb.visionInfo.circleTrack
        # print track
        # if fabs(track.yaw) > 120:
        #     # Trackball
        #     self.lookAt(0, 0)
        #     return self.failure()


        curplat = self.bb.motionInfo.curPlat
        curYaw = curplat.y

        thresh = 5
        pitch = fabs(track.pitch - curplat.x) > thresh and track.pitch or curplat.x
        yaw = fabs(track.yaw - curplat.y) > thresh and track.yaw or curplat.y

        self.lookAt(pitch, yaw, 10, 10)

        return self.success()

