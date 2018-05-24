#!/usr/bin/env python
"""
Created on: May 2, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

TOPIC: 'reload_config'
cwd: $HOME/humanoid/src/dconfig/scripts
Watching directory: ../dmotion
Launched by dmotion/launch/dmotion.launch
Watch config directory, call `rosparam load` and publish msg when file is changed.
"""

import subprocess
import rospy
import os
from std_msgs.msg import String
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from time import sleep

rospy.init_node('config_watchdog')
ROBOT_ID = rospy.get_param('~RobotId')
PATH = '../%s' % (ROBOT_ID)
MOTION_CONFIG = [
    'motion.yml',
    'motor.yml',
    'kick.yml',
    'setup.yml',
    'pvhipY.yml',
    'goalie.yml'
]

MOTION_TOPIC = '/humanoid/ReloadMotionConfig'
VISION_CONFIG = [
    'localization.yml'
]
VISION_TOPIC = '/humanoid/ReloadVisionConfig'


class Handler(FileSystemEventHandler):
    """File Handler."""

    def __init__(self):
        """Init."""
        super(Handler, self).__init__()
        self.pub_motion = rospy.Publisher(MOTION_TOPIC, String, queue_size=1)
        self.pub_vision = rospy.Publisher(VISION_TOPIC, String, queue_size=1)
        rospy.loginfo("CWD: %s" % os.getcwd())

    def on_modified(self, event):
        """Callback on modified."""
        if not event.is_directory:
            _, filename = os.path.split(event.src_path)
            if filename.split('.')[-1] in ['yml', 'yaml']:
                if (filename in MOTION_CONFIG):
                    rospy.loginfo(
                        'motion: rosparam load %s/dvision/%s' % (PATH, filename))
                    subprocess.call(
                        ['rosparam', 'load', '%s/dmotion/%s' % (PATH, filename), 'dmotion_{}'.format(ROBOT_ID)])
                    sleep(1)
                    self.pub_motion.publish('')
                    rospy.loginfo('config changed, published reload msg')
                elif (filename in VISION_CONFIG):
                    rospy.loginfo(
                        'vision: rosparam load %s/dvision/%s' % (PATH, filename))
                    subprocess.call(
                        ['rosparam', 'load', '%s/dvision/%s' % (PATH, filename), 'dvision_{}'.format(ROBOT_ID)])
                    sleep(1)
                    self.pub_vision.publish('')
                    rospy.loginfo('config changed, published reload msg')
            else:
                rospy.logwarn('%s not yaml file' % event.src_path)


if __name__ == '__main__':
    try:
        event_handler = Handler()
        observer = Observer()
        observer.schedule(event_handler, path=PATH, recursive=True)
        observer.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
