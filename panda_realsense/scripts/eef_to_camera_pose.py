#!/usr/bin/env python

import numpy as np
from subprocess import PIPE, Popen
import time

# listen for transformation
listen_transformation = Popen(["rosrun", "tf", "tf_echo", "/panda_hand", "/camera_estimated_link", "5"], stdout=PIPE, bufsize=1)
time.sleep(1)
listen_transformation.terminate()
lines = listen_transformation.stdout.readlines()

# extract transformation from output
for i, line in enumerate(lines):
    if "At time" in line:
        # transformation from eef to estimated camera pose
        eef_to_camera = np.r_['0,2,0', np.array(lines[i+1].split('[')[1][0:-2].split(", ")).astype(np.float32), np.array(lines[i+3].split('[')[1][0:-2].split(", ")).astype(np.float32)]
        break;

print("transformation from endeffector to camera pose:")
print(eef_to_camera)

np.savetxt("eef_to_camera.txt", eef_to_camera)
