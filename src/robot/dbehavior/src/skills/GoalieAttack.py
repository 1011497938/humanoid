from DecisionMaking.BehaviorTree.Branch import sequence, selector
from DecisionMaking.BehaviorTree.Task import Action
from skills.ApproachBall import ApproachBall
from skills.Dribble import Dribble
from skills.Kick import Kick
from skills.SeekBall import SeekBall
from skills.FindBallAroundOnce import FindBallAroundOnce
from skills.SeekBall_Fuck import SeekBall_Fuck
from skills.GoToGoaliePoint import GoToGoaliePoint
from headskills.GoalieScanField import GoalieScanField
from headskills.IsBallFarAway import IsBallFarAway

GoalieAttack = sequence(FindBallAroundOnce,
                  ApproachBall,
                  selector(Dribble, Kick))
