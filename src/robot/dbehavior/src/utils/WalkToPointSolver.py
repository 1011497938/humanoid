from Blackboard import getbb
from utils.VecPos import VecPos
from utils.mathutil import *
from utils.VecPos import calc_field_position

NEARBY_SWITCHING_DISTANCE = 30
FARAWAY_SWITCHING_DISTANCE = 40

TOP_X_MAX = 6.0
TOP_THETA_MAX = 5.0

MID_X_MAX = 4.5
MID_THETA_MAX = 10.0

AD_X_MAX = 2.0
AD_THETA_MAX = 10.0

STEP_THETA_MAX = 15.0

THETA_SAFE = 10.0
epso = 1e-10

walkAbility = [TOP_X_MAX, TOP_X_MAX, MID_X_MAX, AD_X_MAX, epso], \
              [epso, TOP_THETA_MAX, MID_THETA_MAX, AD_THETA_MAX, STEP_THETA_MAX]

WALength = len(walkAbility[0])



class WalkToPointSolver(object):
    """
    Walk to point solver, input must be in the same coordinate,
    Refer to Ydd's documentation, if u don't understand just call him: 18768116076.
    
    :param RobotPos: robot's current position (x, y, angle)
    :param DestPos: robot's next destnation 
    
    It's next behavior request.
    forward cm/step, left cm/step, turn deg/step
    """

    def __init__(self):
        """
        For simplicity, we use destination in local coordinate system for calculating.
        """
        self.nearDistance = NEARBY_SWITCHING_DISTANCE
        self.vx = 0
        self.bb = getbb()

    def solveField(self, dest, forceOmni = False):
        """
        :return: [forward, left, turn] 
        """
        self.vx = self.bb.motionInfo.robotCtrl.x

        if dest.length() < self.nearDistance or forceOmni:
            self.nearDistance = FARAWAY_SWITCHING_DISTANCE
            return self.omnidirection(dest)
        else:
            self.nearDistance = NEARBY_SWITCHING_DISTANCE
            return self.fast(dest)

    def solveGlobal(self, destGlobal, forceOmni = False):
        dest = calc_field_position(destGlobal, self.bb.robot_pos)
        return self.solveField(dest, forceOmni)

    def omnidirection(self, localDest):
        mirror = False
        dest = localDest.copy()

        if dest.y < 0:
            dest.y = -dest.y
            dest.z = -dest.z
            mirror = True

        l = min(AD_X_MAX, dest.length())
        thetaP = dest.slope()
        thetaAttack = dest.z

        if abs(thetaAttack) > AD_THETA_MAX:
            thetaOut = AD_THETA_MAX * sign(thetaAttack)
        else:
            thetaOut = thetaAttack

        tmpAngle = thetaP - thetaOut
        xOut = l * cosd(tmpAngle)
        yOut = l * sind(tmpAngle)

        if mirror:
            yOut = -yOut
            thetaOut = -thetaOut

        return xOut, yOut, thetaOut

    def fast(self, localDest):
        sx_star, sy_star, st_star = 0, 0, 0
        mirror = False
        dest = localDest.copy()

        if dest.y < 0:
            dest.y = -dest.y
            dest.z = -dest.z
            mirror = True

        xInput = self.vx
        thetaP = dest.slope()
        thetaAttack = dest.z

        thetaSafe = min(thetaP, THETA_SAFE)
        thetaAttackP = thetaAttack - thetaP

        aDest = dest.copy()
        aDest.rotate(90)

        if thetaAttackP > thetaP:
            thetaMax = thetaP
        elif thetaAttackP > thetaSafe:
            thetaMax = thetaAttackP
        else:
            thetaMax = thetaSafe

        thetaMax = min(thetaMax, thetaSafe)
        thetaMax = min(thetaMax, thetaP)

        dt = 1
        stepnumMin = 999999

        i = 0
        if abs(thetaMax) < TOP_THETA_MAX:
            sx_star, sy_star, st_star = TOP_X_MAX, 0, thetaMax
        else:
            cross_i = VecPos()
            while i < thetaMax:
                a_i = aDest.copy()
                a_i.rotate(i)
                b_i = a_i.dot(dest)
                r_i = b_i / (a_i.y - a_i.length() + epso)

                sx, sy, st = self.getx_max(r_i)
                st = min(thetaP, st)
                c_i = i + thetaP
                cross_i.x = r_i * sind(c_i)
                cross_i.y = r_i * (1 - cosd(c_i))
                line_i = (dest - cross_i).copy()

                k1 = 15.0
                k2 = 10.0
                stepnum_arc = c_i / (st == 0 and epso or st)
                stepnum_line = line_i.length() / TOP_X_MAX
                if thetaP < AD_THETA_MAX + 20:
                    k2 = 25.0

                stepnum_speed = abs(max(-sx + xInput, 0)) / TOP_X_MAX * k1 * thetaP * pi / 180 + \
                                abs(TOP_X_MAX - sx) * k2 / TOP_X_MAX

                stepnum = stepnum_arc + stepnum_line + stepnum_speed
                if stepnumMin > stepnum:
                    stepnumMin = stepnum
                    sx_star, sy_star, st_star = sx, sy, st

                i += dt

        if mirror:
            st_star = -st_star
            # sy_star = -sy_star

        # print 'fast', sx_star, sy_star, st_star, localDest.x, localDest.y, localDest.z
        return sx_star, sy_star, st_star

    def getx_max(self, r):
        """get max sx and max st"""
        a_norm = [[0 for col in range(0, WALength - 1)]
                  for row in range(0, WALength)]
        b = [0 for col in range(0, len(walkAbility[0]) - 1)]

        for i in range(0, 4):
            a_norm[1][i] = walkAbility[1][i + 1] - walkAbility[1][i] + epso
            a_norm[0][i] = -walkAbility[0][i + 1] + walkAbility[0][i] + epso
            b[i] = a_norm[0][i] * walkAbility[1][i] + a_norm[1][i] * walkAbility[0][i]
        rec = [walkAbility[0][i] * 1.0 / walkAbility[1][i]
               * 180 / pi for i in range(0, len(walkAbility[0]))]
        for i in range(0, 4):
            if r <= rec[i]:
                if abs(a_norm[1][i]) < epso:
                    gait_sx = r * pi * walkAbility[1][i] / 180
                    gait_st = walkAbility[1][i]
                else:
                    gait_st = b[i] / a_norm[1][i] / \
                              (pi * r / 180 + a_norm[0][i] / a_norm[1][i])
                    gait_sx = (b[i] - a_norm[0][i] * gait_st) / a_norm[1][i]

        gait_sy = 0
        return gait_sx, gait_sy, gait_st
