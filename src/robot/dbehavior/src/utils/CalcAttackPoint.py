from utils.VecPos import VecPos
from utils.mathutil import *
from Blackboard import getbb

# config
magic = 40
DOGE_POINT = VecPos(magic, 0)
DOGE_POINT_UP = VecPos(magic, magic)
DOGE_POINT_DOWN = VecPos(magic, -magic)
DOGE_ANGLE = degree_between(DOGE_POINT, DOGE_POINT_UP)

rub = None
final_dest = None
dest = None
bb = getbb()
cfg = bb.parameters

def get_attack_result():
    global final_dest, dest, rub
    return final_dest, dest, rub

def get_rub():
    global rub
    if not rub:
        raise Exception('rub not initialised, call calc_attack_point first')
    return rub

def calc_attack_point():
    global rub, final_dest, dest, bb

    robot_pos = bb.robot_pos
    ball_global = bb.ball_global
    ball_field = bb.ball_field
    enable_kick = bb.enable_kick
    attack_target = bb.attack_target

    final_dest = calc_final_dest(enable_kick, ball_field, ball_global, attack_target)
    rub = calc_rub(ball_global, attack_target, robot_pos)
    theta = degree_between(DOGE_POINT, rub)

    if rub.x < -10:
        if theta > DOGE_ANGLE:
            dest = calc_doge_point(rub, robot_pos, ball_global, attack_target, final_dest, 'up')
        else:
            dest = calc_doge_point(rub, robot_pos, ball_global, attack_target, final_dest, 'down')
    else:
        dest = final_dest

    if get_dis(robot_pos, dest) < 40 or get_dis(robot_pos, final_dest) < 40:
        dest = final_dest

    return final_dest, dest, rub

def calc_final_dest(enable_kick, ball_field, ball_global, target):
    theta = angle_between(target, ball_global)

    closer_to_left_foot = ball_field.y > 0

    # FIXME(MWX): may blur
    if not enable_kick or (not cfg.left_kick and not cfg.right_kick):
        kick_point = closer_to_left_foot and cfg.left_kick_point or cfg.right_kick_point
    else:
        kick_point = cfg.left_kick and cfg.left_kick_point or cfg.right_kick_point

    res = VecPos()
    res.x = ball_global.x + kick_point.x * cos(theta)
    res.y = ball_global.y + kick_point.x * sin(theta)

    theta2 = angle_between(target, res)
    res.z = degrees(theta2 + PI)
    return res

def calc_rub(ball_global, target, robot_pos):
    b2r = bb.robot_pos - bb.ball_global
    theta = degree_between(target, ball_global)
    b2r.rotate(-theta)
    return b2r

def calc_doge_point(rub, robot_pos, ball_global, target, final_dest, side):
    doge = side is 'up' and DOGE_POINT_UP or DOGE_POINT_DOWN

    if -30 < rub.x < 0 and abs(rub.y) < 20:
        doge = side is 'up' and VecPos(0, 20) or VecPos(0, -20)

    theta = angle_between(ball_global, target)
    dogex = ball_global.x + doge.y * sin(theta) - doge.x * cos(theta)
    dogey = ball_global.y - doge.y * cos(theta) + doge.x * sin(theta)
    d = VecPos(dogex, dogey)
    angle = degree_between(d, final_dest)

    return VecPos(dogex, dogey, angle)
