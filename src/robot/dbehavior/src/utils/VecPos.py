from math import sqrt, atan2, pi, sin, cos, radians
from mathutil import cosd, sind, angle_normalization
from geometry_msgs.msg import Vector3

class VecPos(object):
    def __init__(self, x = 0, y = 0, angle = 0):
        self.x = x
        self.y = y
        self.z = angle

    @classmethod
    def fromVector3(cls, vec):
        return cls(vec.x, vec.y, vec.z)

    def __str__(self):
        return "[{}, {}, {}]".format(self.x, self.y, self.z)

    def __add__(self, pos_):
        tmp = VecPos(0, 0)
        tmp.x = self.x + pos_.x
        tmp.y = self.y + pos_.y
        return tmp

    def __sub__(self, pos_):
        tmp = VecPos(0, 0)
        tmp.x = self.x - pos_.x
        tmp.y = self.y - pos_.y
        return tmp

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y

    def __eq__(self, pos_):
        tmp = VecPos(0, 0)
        tmp.x = pos_.x
        tmp.y = pos_.y
        return tmp

    def length(self):
        return sqrt(self.x * self.x + self.y * self.y)

    def distance(self, dest):
        return sqrt((dest.x - self.x) * (dest.x - self.x) +
                    (dest.y - self.y) * (dest.y - self.y))

    def slope(self):
        return atan2(self.y, self.x) * 180 / pi

    def copy(self):
        return VecPos(self.x, self.y, self.z)

    def rotate(self, angle):
        x = self.x
        y = self.y
        self.x = x * cosd(angle) - y * sind(angle)
        self.y = x * sind(angle) + y * cosd(angle)
        return self

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def toVector3(self):
        return Vector3(self.x, self.y, self.z)

    def mirror(self):
        return VecPos(-self.x, -self.y, angle_normalization(self.z + 180))

def make_vecpos(pos):
    return VecPos(pos.x, pos.y)

def calc_global_position(field_pos, robot_state):
    """
    From robot's global position and object's field position, get object's global position
    """
    tmp = VecPos(field_pos.x, field_pos.y)
    tmp.rotate(robot_state.z)
    return VecPos(tmp.x + robot_state.x, tmp.y + robot_state.y)


def calc_field_position(global_pos, robot_state):
    """
    get the field position of an object with it's global position and robot's global position
    """
    tmp = VecPos(global_pos.x - robot_state.x, global_pos.y - robot_state.y)
    tmp.rotate(-robot_state.z)
    tmp.z = angle_normalization(global_pos.z - robot_state.z)
    return tmp
