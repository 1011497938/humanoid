
from dmsgs.msg import BodyCommand
from dmsgs.msg import HeadCommand
from dmsgs.msg import ActionCommand


def head(pitch = 0, yaw = 0, pitchSpeed = 2, yawSpeed = 2):
    """
    HeadCommand defined as
    ===
    float32 pitch
    float32 yaw
    float32 pitchSpeed
    float32 yawSpeed
    """
    return HeadCommand(pitch, yaw, pitchSpeed, yawSpeed)

def crouch():
    """
    BodyCommand defined as [x, y, t, gaitType]
    ===
    """
    return BodyCommand(0, 0, 0, BodyCommand.CROUCH)

def standup():
    return BodyCommand(0, 0, 0, BodyCommand.STANDUP)

def walk(forward = 0, left = 0, turn = 0):
    return BodyCommand(forward, left, turn, BodyCommand.WENXI)

def goalieMid():
    return BodyCommand(0, 0, 0, BodyCommand.GOALIEMID)

def goalieLeft():
    return BodyCommand(0, 0, 0, BodyCommand.GOALIELEFT)

def goalieRight():
    return BodyCommand(0, 0, 0, BodyCommand.GOALIERIGHT)

def kickLeft():
    return BodyCommand(0, 0, 0, BodyCommand.KICKLEFT)

def kickRight():
    return BodyCommand(0, 0, 0, BodyCommand.KICKRIGHT)


if __name__ == '__main__':
    h = head(1, 2, 3, 4)
    c = crouch()
    print h, c
