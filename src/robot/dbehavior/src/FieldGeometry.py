from utils.VecPos import VecPos

StrikerLeftPosKickoff = VecPos(-50, 0, 0)

StrikerLeftPos = VecPos(-150, 75, 0)

DefenderLeftPos = VecPos(-300, 0, 0)

MidfielderLeftPos = VecPos(-150, 200, 0)

GoalieLeftPos = VecPos(-400, 0, 0)


StrikerRightPos = StrikerLeftPos.mirror()
DefenderRightPos = DefenderLeftPos.mirror()
MidfielderRightPos = MidfielderLeftPos.mirror()
GoalieRightPos = GoalieLeftPos.mirror()
