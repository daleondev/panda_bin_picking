import os 
import time
import signal
import subprocess

from client import log

from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QCursor

node_gpd = None
inputs = []

def start_gpd():
    global node_gpd

    QApplication.setOverrideCursor(QCursor(Qt.WaitCursor))

    log("Starting Grasp Pose Detection")
    node_gpd = subprocess.Popen(["rosrun", "gpd_ros", "detect_grasps",  "_config_file:=" + os.path.join(os.path.abspath(os.path.dirname(__file__)), "../../config/gpd_params.cfg"),
                                                                        "_cloud_type:=0", 
                                                                        "_cloud_topic:=/panda_bin_picking/cloud_gpd", 
                                                                        "_rviz_topic:=plot_grasps"])

    time.sleep(1)
    if node_gpd.poll() is None:
        log("<font color=#a0ffa0>[Succeeded]</font>")
    else:
        log("<font color=#ffa0a0>[Failed]</font>")

    QApplication.restoreOverrideCursor()

def stop_gpd():
    global node_gpd

    QApplication.setOverrideCursor(QCursor(Qt.WaitCursor))

    log("Stopping Grasp Pose Detection")
    os.killpg(os.getpgid(node_gpd.pid), signal.SIGTERM)
    time.sleep(1)
    if node_gpd.poll() is not None:
        log("<font color=#a0ffa0>[Succeeded]</font>")
    else:
        log("<font color=#ffa0a0>[Failed]</font>")

    QApplication.restoreOverrideCursor()

def load_settings():
    lines = open(os.path.join(os.path.abspath(os.path.dirname(__file__)), "../../config/gpd_params.cfg"), "r").readlines()
    for line in lines:
        if line[0] == '#':
            continue

        parts = line.split('#')[0].split('=')
        if "finger_width" in parts[0]:
            inputs[0].setText(parts[1].replace('\n', '')[1:])

        elif "hand_outer_diameter" in parts[0]:
            inputs[1].setText(parts[1].replace('\n', '')[1:])

        elif "hand_depth" in parts[0]:
            inputs[2].setText(parts[1].replace('\n', '')[1:])

        elif "hand_height" in parts[0]:
            inputs[3].setText(parts[1].replace('\n', '')[1:])


        elif "volume_width" in parts[0]:
            inputs[4].setText(parts[1].replace('\n', '')[1:])

        elif "volume_depth" in parts[0]:
            inputs[5].setText(parts[1].replace('\n', '')[1:])

        elif "volume_height" in parts[0]:
            inputs[6].setText(parts[1].replace('\n', '')[1:])


        elif "num_selected" in parts[0]:
            inputs[7].setText(parts[1].replace('\n', '')[1:])

        elif "num_samples" in parts[0]:
            inputs[8].setText(parts[1].replace('\n', '')[1:])

        elif "num_orientations" in parts[0]:
            inputs[9].setText(parts[1].replace('\n', '')[1:])

        elif "num_finger_placements" in parts[0]:
            inputs[10].setText(parts[1].replace('\n', '')[1:])

        elif "deepen_hand" in parts[0]:
            inputs[11].setText(parts[1].replace('\n', '')[1:])


        elif "workspace" in parts[0]:
            inputs[12].setText(parts[1].replace('\n', '')[1:])

        elif "filter_approach_direction" in parts[0]:
            inputs[13].setText(parts[1].replace('\n', '')[1:])

        elif "direction" in parts[0]:
            inputs[14].setText(parts[1].replace('\n', '')[1:])

        elif "thresh_rad" in parts[0]:
            inputs[15].setText(parts[1].replace('\n', '')[1:])

def write_settings():
    file = open(os.path.join(os.path.abspath(os.path.dirname(__file__)), "../../config/gpd_params.cfg"), "r+")

    lines = []
    for line in file.read().splitlines():
        if line and line[0] == '#':
            lines.append(line)
            continue

        parts = line.split('#')[0].split('=')

        if "finger_width" in parts[0]:
            lines.append(parts[0] + "= " + inputs[0].displayText())

        elif "hand_outer_diameter" in parts[0]:
            lines.append(parts[0] + "= " + inputs[1].displayText()) 

        elif "hand_depth" in parts[0]:
            lines.append(parts[0] + "= " + inputs[2].displayText()) 

        elif "hand_height" in parts[0]:
            lines.append(parts[0] + "= " + inputs[3].displayText()) 


        elif "volume_width" in parts[0]:
            lines.append(parts[0] + "= " + inputs[4].displayText()) 

        elif "volume_depth" in parts[0]:
            lines.append(parts[0] + "= " + inputs[5].displayText()) 

        elif "volume_height" in parts[0]:
            lines.append(parts[0] + "= " + inputs[6].displayText()) 


        elif "num_selected" in parts[0]:
            lines.append(parts[0] + "= " + inputs[7].displayText()) 

        elif "num_samples" in parts[0]:
            lines.append(parts[0] + "= " + inputs[8].displayText()) 

        elif "num_orientations" in parts[0]:
            lines.append(parts[0] + "= " + inputs[9].displayText()) 

        elif "num_finger_placements" in parts[0]:
            lines.append(parts[0] + "= " + inputs[10].displayText()) 

        elif "deepen_hand" in parts[0]:
            lines.append(parts[0] + "= " + inputs[11].displayText()) 


        elif "workspace" in parts[0]:
            lines.append(parts[0] + "= " + inputs[12].displayText()) 

        elif "filter_approach_direction" in parts[0]:
            lines.append(parts[0] + "= " + inputs[13].displayText()) 

        elif "direction" in parts[0]:
            lines.append(parts[0] + "= " + inputs[14].displayText()) 

        elif "thresh_rad" in parts[0]:
            lines.append(parts[0] + "= " + inputs[15].displayText()) 

        else:
            lines.append(line)

    file.seek(0)
    file.truncate()
    file.write('\n'.join(lines))

    # stop_gpd()
    start_gpd()