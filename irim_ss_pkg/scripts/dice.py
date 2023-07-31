#!/usr/bin/env python

import random
import os
import rospkg
import time

# time.sleep(15)

rp = rospkg.RosPack()
path = rp.get_path('irim_ss_pkg')

urdf_file = path + '/xacro/Extra/dice.urdf'

# print(urdf_file)

rx = random.uniform( 0.0, 0.15)
ry = random.uniform(-0.2, 0.2)

RA = random.uniform(-2.0, 2.2)
RB = random.uniform(-2.0, 2.2)
RC = random.uniform(-2.0, 2.2)


cmd = "rosrun gazebo_ros spawn_model -urdf -file " + urdf_file + " -model dice -z 0.9 -x " + str(rx) + " -y " + str(ry) + " -R " + str(RA)  + " -P " + str(RB)  + " -Y " + str(RC) 

print(cmd)

os.system(cmd)
