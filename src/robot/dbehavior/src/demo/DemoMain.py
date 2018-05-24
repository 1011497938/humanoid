from utils.actioncommand import *
from DecisionMaking.BehaviorTree.Task import Action
import termios, tty, sys, select

msg = """
Reading from the keyboard  and Publishing to Motion!
---------------------------
Moving around:
        w
   a    s    d

x : turn around
z : turn left; c : turn right
1: stand
2: step

q : left kick; e : right kick

i : golie left; o : goalie mid; p : goalie right
j : set up from back down
k : set up from front down

r/anything else : crounch

,/. : increase/decrease only linear speed by 10%
</> : increase/decrease only angular speed by 10%

h : print help page

CTRL-C to quit
"""

moveBindings = {
    'w': (3, 0, 0),
    'a': (0, 1, 0),
    's': (-1, 0, 0),
    'd': (0, -1, 0),
    'x': (0, 0, 1),
    'z': (1, 0, 1),
    'c': (1, 0, -1),
}

speedBindings = {
    ',': (1.1, 1),
    '.': (.9, 1),
    '<': (1, 1.1),
    '>': (1, .9),
}

kickBindings = {
		'q': kickLeft(),
		'e': kickRight()
		}

goalieBindings = {
		'i': goalieLeft(),
		'o': goalieMid(),
		'p': goalieRight(),
		}


goalieBinding = 'm'
standupBackBinding = 'j'
standupFrontBinding = 'k'

helpBinding = 'h'

standBinding = '1'
stepBinding = '2'

settings = termios.tcgetattr(sys.stdin)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class DemoMain(Action):
    def init(self):
        print msg

    def tick(self):
        speed = 1.5
        turn = 2
        pitch = 0
        yaw = 0
        key = getKey()

        self.lookAt(pitch, yaw)

        if key in moveBindings.keys():
            x = moveBindings[key][0] * speed
            y = moveBindings[key][1] * speed
            th = moveBindings[key][2] * turn
            if abs(x) > 1 or abs(y) > 1 or abs(th) > 1:
                self.walk(x, y, th)
        elif key in kickBindings.keys():
            self.do(kickBindings[key])
        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]
            print vels(speed, turn)
        elif key == helpBinding:
            print msg
        elif key in goalieBindings.keys():
            self.do(goalieBindings[key])
        elif key == standupBackBinding:
            pass
        elif key == standupFrontBinding:
            pass
        elif key == standBinding:
            self.stand()
        elif key == stepBinding:
            self.walk(0, 0, 0)
        else:
            self.crouch()
