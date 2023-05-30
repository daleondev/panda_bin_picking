#!/usr/bin/env python

import sys
import os
import rospy

sys.path.insert(1, os.path.dirname(os.path.abspath(__file__)) + "/gui")
from window import *

if __name__ == "__main__":
    rospy.init_node("panda_bin_picking_gui")

    app = QApplication( sys.argv )
    app.setApplicationName("Panda Bin Picking")

    window = createWindow()
    window.show()

    start_gpd()

    app.exec_()
   
    PandaBinPickingClient.shutDown()

    stop_gpd()