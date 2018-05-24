from DecisionMaking.BehaviorTree.Branch import parallel
from DecisionMaking.BehaviorTree.Branch import selector
from headskills.LookAround import LookAround
from skills.WalkAroundPenalty import WalkAroundPenalty
from skills.SeekBall import SeekBall
from skills.FindBallAroundOnce import FindBallAroundOnce

SeekBall_Fuck = selector(FindBallAroundOnce,
                         parallel(LookAround, WalkAroundPenalty))
