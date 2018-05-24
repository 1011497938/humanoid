#!/usr/bin/env python

# Init
import sys
import rospy
sys.dont_write_bytecode = True
print sys.version
rospy.init_node('behavior_node')

import os
import Log
import pkgutil
from DecisionMaking.BehaviorTree.Task import Task
from utils.CalcAttackPoint import calc_attack_point
from Brain import Brain
from Blackboard import getbb

def getSkill():
    bb = getbb()
    skill = bb.parameters.skill
    behaivorPackages = ["roles", "skills", "demo", "headskills", "states"]
    foundSkill = False
    skillDir = os.getcwd()
    skillClass = None
    for package in behaivorPackages:
        modules = ['{}/{}'.format(skillDir, package)]
        allName = [name for _, name, _ in pkgutil.iter_modules(modules)]
        if skill not in allName:
            continue

        skillModule = __import__('{}.{}'.format(package, skill), fromlist=[skill])
        skillClass = getattr(skillModule, skill)
        foundSkill = True
        print '\n[python] Module --- {} --- imported\n'.format(skill)

    if not foundSkill:
        raise ImportError('Can not find skill: {}'.format(skill))
    elif not issubclass(skillClass, Task):
        raise TypeError('Found skill is not a Task')

    return skillClass()

def main():
    skillInstance = getSkill()
    rate = rospy.Rate(60)
    bb = getbb()
    brain = Brain()
    while not rospy.is_shutdown():
        bb.resetCmd()
        bb.checkMotionConnection()
        # brain
        brain.tick()
        skillInstance.tick()
        bb.publish()
        rate.sleep()

if __name__ == '__main__':
    main()
