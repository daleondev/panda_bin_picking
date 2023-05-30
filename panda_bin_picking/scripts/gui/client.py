import rospy

from std_srvs.srv import Trigger
from std_msgs.msg import Empty

from enum import Enum
from concurrent.futures import ThreadPoolExecutor

from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QCursor

SHUTDOWN_TOPIC          = "/panda_bin_picking/shutdown"

CAPTURE_SERVICE         = "/panda_bin_picking/capture"
DETECT_SERVICE          = "/panda_bin_picking/detect"
PLAN_APPROACH_SERVICE   = "/panda_bin_picking/plan_approach"
APPROACH_SERVICE        = "/panda_bin_picking/approach"
PLAN_PICK_SERVICE       = "/panda_bin_picking/plan_pick"
PICK_SERVICE            = "/panda_bin_picking/pick"
PLAN_LIFT_SERVICE       = "/panda_bin_picking/plan_lift"
LIFT_SERVICE            = "/panda_bin_picking/lift"
PLAN_LIFT_ALT_SERVICE   = "/panda_bin_picking/plan_lift_alt"
LIFT_ALT_SERVICE        = "/panda_bin_picking/lift_alt"
PLAN_PLACE_SERVICE      = "/panda_bin_picking/plan_place"
PLACE_SERVICE           = "/panda_bin_picking/place"

buttons = []
skip_buttons = []
output  = []

class Action(Enum):
    Repeat          = 0
    Back            = 1
    Auto            = 2
    Next            = 3
    Shutdown        = 4
    # Reset           = 5
    

def log(msg):
    output[0].append(msg)

def disableButtons():
    for i in range(0, 5):
        if not i == Action.Shutdown.value and not i == Action.Auto.value: 
            buttons[i].setEnabled(False)

    QApplication.setOverrideCursor(QCursor(Qt.WaitCursor))

def enableButtons(fut):
    if fut.result().success:
        log("<font color=#a0ffa0>[Succeeded]</font>")
    else:
        log("<font color=#ffa0a0>[Failed]</font>")

    QApplication.restoreOverrideCursor()

    for i in range(0, 5):
        if not i == Action.Shutdown.value and not i == Action.Auto.value and i not in skip_buttons:
            buttons[i].setEnabled(True)    

    if not fut.result().success:
        buttons[Action.Next.value].setEnabled(False)

class PandaBinPickingClient:
    _shut_down          = rospy.Publisher(SHUTDOWN_TOPIC, Empty, queue_size=1)

    _capture            = rospy.ServiceProxy(CAPTURE_SERVICE, Trigger)
    _detect             = rospy.ServiceProxy(DETECT_SERVICE, Trigger)
    _plan_approach      = rospy.ServiceProxy(PLAN_APPROACH_SERVICE, Trigger)
    _approach           = rospy.ServiceProxy(APPROACH_SERVICE, Trigger)
    _plan_pick          = rospy.ServiceProxy(PLAN_PICK_SERVICE, Trigger)
    _pick               = rospy.ServiceProxy(PICK_SERVICE, Trigger)
    _plan_lift          = rospy.ServiceProxy(PLAN_LIFT_SERVICE, Trigger)
    _lift               = rospy.ServiceProxy(LIFT_SERVICE, Trigger)
    _plan_lift_alt      = rospy.ServiceProxy(PLAN_LIFT_ALT_SERVICE, Trigger)
    _lift_alt           = rospy.ServiceProxy(LIFT_ALT_SERVICE, Trigger)
    _plan_place         = rospy.ServiceProxy(PLAN_PLACE_SERVICE, Trigger)
    _place              = rospy.ServiceProxy(PLACE_SERVICE, Trigger)

    _executor           = ThreadPoolExecutor(max_workers=1)

    @staticmethod
    def capture():
        # rospy.wait_for_service(CAPTURE_SERVICE)
        disableButtons()
        log("Capturing Pointclouds")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._capture.call) 
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def detect():
        disableButtons()
        log("Detecting grasp poses")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._detect.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def planApproach():
        disableButtons()
        log("Planning approach path")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._plan_approach.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def approach():
        disableButtons()
        log("Approaching object")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._approach.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def planPick():
        disableButtons()
        log("Planning pick path")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._plan_pick.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def pick():
        disableButtons()
        log("Picking object")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._pick.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def planLift():
        disableButtons()
        log("Planning lift path")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._plan_lift.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def lift():
        disableButtons()
        log("Lifting object")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._lift.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def planLiftAlt():
        disableButtons()
        log("Planning alternative lift path")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._plan_lift_alt.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def liftAlt():
        disableButtons()
        log("Lifting object")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._lift_alt.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def planPlace():
        disableButtons()
        log("Planning place path")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._plan_place.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def place():
        disableButtons()
        log("Placing object")
        future = PandaBinPickingClient._executor.submit(PandaBinPickingClient._place.call)
        future.add_done_callback(enableButtons)
        return future

    @staticmethod
    def shutDown():
        disableButtons()
        msg = Empty()
        PandaBinPickingClient._shut_down.publish(msg)