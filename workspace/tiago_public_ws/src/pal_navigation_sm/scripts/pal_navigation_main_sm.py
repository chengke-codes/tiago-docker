#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_navigation_main_sm.py
#
# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# This is a fake version of pal_navigation_sm. The complete version implements a real SMACH able to manage multiple maps,
# change dynamically between mapping and localization and many other features.
#
# Authors:
#   * Ricardo Tellez
#   * Siegfried-A. Gevatter
#   * Jordi Pages
#

import roslib; roslib.load_manifest('pal_navigation_sm')
import rospy
import rosparam
import os
import shutil
import subprocess
import time

import rospkg
from pal_navigation_msgs.srv import SaveMap, SaveMapResponse
from pal_python import pal_launch

DEFAULT_TRANSFORMATION = """<MapTransformationConfig scale="1.0">
  <location3D>
    <translation x="0"  y="0"  z="0" ></translation>
    <matrixRot rodrigues_x="0"  rodrigues_y="0"  rodrigues_z="0" ></matrixRot>
    <rotationRollPitchYaw roll="0"  pitch="0"  yaw="0" ></rotationRollPitchYaw>
  </location3D>
</MapTransformationConfig>
"""

class MapsManagerService():

    def __init__(self):

        self._save_map_srv = rospy.Service('pal_map_manager/save_map', SaveMap, self._save_map_request)
        self._robot = rospy.get_param("/pal_robot_info/type", "tiago")

    def _save_map_request(self, req):

        rospy.logerr("robot: {}".format(self._robot))

        map_path = os.path.expanduser('~') + '/.pal/' + self._robot + '_maps/'

        if req.directory:
            map_name = req.directory
        else:
            timestamp = time.localtime()
            map_name = time.strftime('%F_%H%M%S', timestamp)

        full_map_path = map_path + "configurations/" + map_name
        short_map_path = map_path + 'config'

        shutil.rmtree(full_map_path, ignore_errors=True)
        subprocess.call(['rm', '-rf', short_map_path])
        
        os.makedirs(full_map_path)
        subprocess.call(['ln', '-s', full_map_path, map_path + 'config'])

        rospy.logerr("Saving map: " + full_map_path)

        # Save map (image and meta-data).
        filename = 'submap_0'
        pal_launch.execute_command('rosrun', 'map_server', 'map_saver', '-f', filename)

        # Update /mmap/numberOfSubMaps parameter
        # [from saverFunctions::configSaver]
        rospy.set_param('/mmap/numberOfSubMaps', 1)
        map_data_file = os.path.join(full_map_path + '/', 'mmap.yaml')
        if rospy.has_param('/mmap'):
            rosparam.dump_params(map_data_file, '/mmap')
        else:
            try:
                os.remove(map_data_file)
            except OSError:
                pass
            map_data_file = None

        # Move map to maps folder
        # (this is required to have the image relative path in map.yaml):
        shutil.move(filename + '.pgm', full_map_path)
        shutil.move(filename + '.yaml',full_map_path + '/map.yaml')

        # Create nice map
        shutil.copyfile(os.path.join(full_map_path, 'submap_0.pgm'), os.path.join(full_map_path, 'map.pgm'))
        with open(os.path.join(full_map_path, 'transformation.xml'), 'w') as transf:
            transf.write(DEFAULT_TRANSFORMATION)

        # Set permisions
        self._chmod(map_path, 0777)

        # Save initial pose of the robot in the map
        rospack = rospkg.RosPack()
        shutil.copyfile(rospack.get_path('pal_navigation_sm') + '/config/pose.yaml', map_path+'../pose.yaml')

        return SaveMapResponse(True, map_name, map_path, 'Map saved: %s' % map_name)

    def _chmod(self, path, mode):
        """
        Change the permissions of `path' recursively, including
        `path' itself.
        """
        os.chmod(path, mode)
        for root, dirs, files in os.walk(path):
            for d in dirs:
                os.chmod(os.path.join(root, d), mode)
            for f in files:
                os.chmod(os.path.join(root, f), mode)




def main():
    rospy.init_node('navigation_sm', log_level=rospy.ERROR)

    MapsManagerService()

    rospy.spin()


if __name__ == "__main__":
    main()
