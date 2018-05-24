# @Author: Yusu Pan <yuthon>
# @Date:   2017-07-25T11:06:47+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: AvoidObstacleTest.py
# @Last modified by:   yuthon
# @Last modified time: 2017-07-25T11:06:48+08:00
# @Copyright: ZJUDancer

from skills.AvoidObstacle import AvoidObstacle
from skills.Attack import Attack
from DecisionMaking.BehaviorTree.Branch import sequence, selector, parallel


AvoidObstacleTest = selector(AvoidObstacle, Attack)
