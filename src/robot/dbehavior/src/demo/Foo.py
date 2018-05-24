from DecisionMaking.BehaviorTree.Task import Action

class Foo(Action):
    def init(self):
        pass

    def tick(self):
        self.lookAt(45, 0)

        print '------------------'
        print self.bb.GCInfo

        # if self.bb.GCInfo:
        #     print self.bb.GCInfo.kickoff
        # info1 = self.bb.getTeamInfo(1)
        # if info1:
        #     print info1
        # print self.bb.behaviorInfo.time_to_reach_ball, self.bb.closest_to_ball()

        if self.bb.GCInfo:
            print self.bb.GCInfo


# time recv_timestamp
# uint8 player_number
# bool incapacitated
#
# # behavior info
# uint8 current_role
# float32 time_to_reach_ball
# geometry_msgs/Vector3 dest
# geometry_msgs/Vector3 final_dest
#
# # vision info
# bool see_ball
# geometry_msgs/Vector3 robot_pos
# geometry_msgs/Vector3 ball_field
# geometry_msgs/Vector3 ball_global
