from DecisionMaking.BehaviorTree.Task import Task, Action
from DecisionMaking.BehaviorTree.Decorator import condition, guard
from dmsgs.msg import GCInfo
import rospy
from states.GCStates import Initial, Set, Playing, Finished
from states.Ready import Ready
from Blackboard import attackRight
from utils.VecPos import VecPos
from Timer import Timer
from headskills.TrackBall import TrackBall


class GCPlay(Action):
    def init(self):
        self.last_state = None
        self.initial = Initial()
        self.ready = Ready()
        self.set = Set()
        self.playing = Playing()
        self.finished = Finished()
        self.cur_state = None
        self.freeKickTimer = Timer(10)
        self.lastFreeKickBallPos = VecPos()
        self.enemy_freekick = False
        self.trackBall = TrackBall()

    def tick(self):
        #print 'GCPlay'
        self.trackBall.tick()
        gc_info = self.bb.GCInfo

        if gc_info is None or not gc_info.connected:
            self.playing.tick()
        else:
            if gc_info.ourIndirectFreeKick or gc_info.ourDirectFreeKick or gc_info.ourPenaltyKick:
                if gc_info.state2Freeze:
                    self.trackBall.tick()
                    self.crouch()
                elif gc_info.state2Ready:
                    self.trackBall.tick()
                    if self.bb.closest_to_ball():
                        ball = self.bb.ball_global
                        if attackRight():
                            self.gotoGlobal(VecPos(ball.x - 80, ball.y, 0))
                        else:
                            self.gotoGlobal(VecPos(ball.x + 80, ball.y, 0))

            elif gc_info.enemyIndirectFreeKick or gc_info.enemyDirectFreeKick or gc_info.enemyPenaltyKick:
                self.enemy_freekick = True
                self.freeKickTimer.restart()
                self.lastFreeKickBallPos = self.bb.ball_global

                if gc_info.state2Freeze:
                    self.crouch()
                elif gc_info.state2Ready:
                    if self.bb.closest_to_ball():
                        ball = self.bb.ball_global
                        if attackRight():
                            self.gotoGlobal(VecPos(ball.x - 50, ball.y, 0))
                        else:
                            self.gotoGlobal(VecPos(ball.x + 50, ball.y, 0))
                    else:
                        self.playing.tick()

            elif self.enemy_freekick:
                if self.bb.closest_to_ball():
                    if not self.freeKickTimer.finished():
                        # print self.freeKickTimer.elapsed()
                        ball_now = self.bb.ball_global
                        diff = (ball_now - self.lastFreeKickBallPos).length()

                        # print diff

                        if diff < 20:
                            self.crouch()
                            self.gazeBall()
                        else:
                            self.playing.tick()
                    else:
                        self.enemy_freekick = False
                else:
                    self.enemy_freekick = False
            else:
                if gc_info.state is GCInfo.INITIAL:
                    self.cur_state = self.initial

                elif gc_info.state is GCInfo.READY:
                    self.cur_state = self.ready

                elif gc_info.state is GCInfo.SET:
                    self.cur_state = self.set

                elif gc_info.state is GCInfo.PLAYING:
                    self.cur_state = self.playing

                elif gc_info.state is GCInfo.FINISHED:
                    self.cur_state = self.finished

                else:
                    self.cur_state = self.playing

                if gc_info.state != self.last_state:
                    if self.cur_state:
                        self.cur_state.fresh()

                if self.cur_state:
                    self.cur_state.tick()

                self.last_state = gc_info.state



