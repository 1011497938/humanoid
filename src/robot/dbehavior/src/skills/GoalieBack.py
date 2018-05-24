from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel, selector
from headskills.TrackBall import TrackBall
from headskills.GoalieLookDown import GoalieLookDown
from skills.GoToGoaliePoint import GoToGoaliePoint
from utils.mathutil import *

GoalieBack = parallel(GoalieLookDown, GoToGoaliePoint)
