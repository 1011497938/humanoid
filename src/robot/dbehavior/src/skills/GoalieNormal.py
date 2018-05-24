from DecisionMaking.BehaviorTree.Branch import sequence, selector
from DecisionMaking.BehaviorTree.Task import Action
from skills.ApproachBall import ApproachBall
from skills.Dribble import Dribble
from skills.Kick import Kick
from skills.SeekBall import SeekBall
from skills.GoalieMain import GoalieMain
from skills.GoToGoaliePoint import GoToGoaliePoint
from headskills.GoalieScanField import GoalieScanField
from headskills.IsBallFarAway import IsBallFarAway

GoalieNormal = sequence(GoalieScanField, GoalieMain)
