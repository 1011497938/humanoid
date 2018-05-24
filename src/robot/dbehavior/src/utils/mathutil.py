from math import pi, sqrt, atan2, degrees, radians, sin, cos

PI = pi
PI_2 = pi / 2.0
PI_4 = pi / 4.0
PI_1_3 = pi / 3.0

Nan = float('nan')
Inf = float('inf')

def cosd(degree):
    return cos(radians(degree))

def sind(degree):
    return sin(radians(degree))


def get_angle(vector):
    return degrees(atan2(vector.y, vector.x))


def get_dis(x, y):
    dx = x.x - y.x
    dy = x.y - y.y
    return sqrt(dx * dx + dy * dy)


def get_magnitude(vec):
    return sqrt(vec.x * vec.x + vec.y * vec.y)


def angle_between(a, b):
    return atan2(b.y - a.y, b.x - a.x)


def degree_between(a, b):
    return angle_normalization(degrees(angle_between(a, b)))


def angle_between2(a, b, o):
    return angle_between(a - o, b - o)


def degree_between2(a, b, o):
    return angle_normalization(degree_between2(a, b, o))


def angle_normalization(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def abs_angle_diff(angle):
    return abs(angle_normalization(angle))


def sign(v):
    if v > 0:
        return 1.0
    if v < 0:
        return -1.0
    return 0


def radian_to_degree(radian):
    return radian / PI * 180


def degree_to_radian(degree):
    return degree / 180 * PI
