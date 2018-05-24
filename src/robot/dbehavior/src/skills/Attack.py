from DecisionMaking.BehaviorTree.Branch import sequence, selector
from DecisionMaking.BehaviorTree.Task import Action
from skills.ApproachBall import ApproachBall
from skills.Dribble import Dribble
from skills.Kick import Kick
from skills.SeekBall import SeekBall
from skills.SeekBall_Fuck import SeekBall_Fuck

Attack = sequence(SeekBall,
                  ApproachBall,
                  selector(Dribble, Kick))
