from client import *

# from python_qt_binding.QtGui import *
# from python_qt_binding.QtCore import *
# from python_qt_binding.QtWidgets import *

from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QCursor

class State(Enum):
    Home            = 0
    Capture         = 1
    Detect          = 2
    PlanApproach    = 3
    Approach        = 4
    PlanPick        = 5
    Pick            = 6
    PlanLift        = 7
    Lift            = 8
    PlanLiftAlt     = 9
    LiftAlt         = 10
    PlanPlace       = 11
    Place           = 12
    Finished        = 13
    
state = State.Home
out_state = []

def changeState(new_state):
    global state
    state = new_state

    if state == State.Capture:
        out_state[0].setText("-")
    else:
        out_state[0].setText(str(State(state.value-1)).split(".")[1])

    out_state[1].setText(str(state).split(".")[1])

    if state == State.Place:
        out_state[2].setText("Capture")
    else:
	out_state[2].setText(str(State(state.value+1)).split(".")[1])

    del skip_buttons[:]
    if state.value >= State.Pick.value:
        skip_buttons.append(Action.Back.value)
        if state != State.PlanPlace and state != State.PlanLift:
            skip_buttons.append(Action.Repeat.value)
    elif state == State.Capture:
        skip_buttons.append(Action.Back.value)

def stateMachine(action):
    global state
    global future   

    if action == Action.Shutdown:
        state = State.Home                
        PandaBinPickingClient.shutDown()
        QApplication.exit()

    # if action == Action.Reset:
    #     changeState(State.Home)        
    #     buttons[Action.Back.value].setEnabled(False)
    #     buttons[Action.Repeat.value].setEnabled(False)

    if state == State.Home:
        if action == Action.Next:
            changeState(State.Capture)               
            buttons[Action.Back.value].setEnabled(True)
            future = PandaBinPickingClient.capture()         

        return        

    if state == State.Capture:
        if action == Action.Next and future.result().success:
            changeState(State.Detect)           
            future = PandaBinPickingClient.detect()        

        if action == Action.Repeat:            
            future = PandaBinPickingClient.capture()

        return

    if state == State.Detect:
        if action == Action.Next and future.result().success:
            changeState(State.PlanApproach)
            future = PandaBinPickingClient.planApproach()
            
        if action == Action.Repeat:
            future = PandaBinPickingClient.detect()
            
        if action == Action.Back:
            changeState(State.Capture)
            buttons[Action.Back.value].setEnabled(False)

        return

    if state == State.PlanApproach:
        if action == Action.Next and future.result().success:
            changeState(State.Approach)         
            future = PandaBinPickingClient.approach()
            
        if action == Action.Repeat:            
            future = PandaBinPickingClient.planApproach()
            
        if action == Action.Back:
            changeState(State.Detect)

        return

    if state == State.Approach:
        if action == Action.Next and future.result().success:
            changeState(State.PlanPick)          
            future = PandaBinPickingClient.planPick()  

        if action == Action.Back:
            changeState(State.PlanApproach)

        return

    if state == State.PlanPick:
        if action == Action.Next and future.result().success:
            changeState(State.Pick)            
            future = PandaBinPickingClient.pick()
            
        if action == Action.Repeat:            
            future = PandaBinPickingClient.planPick()
            
        if action == Action.Back:
            changeState(State.Approach)

        return

    if state == State.Pick:
        if action == Action.Next and future.result().success:
            changeState(State.PlanLift)           
            future = PandaBinPickingClient.planLift() 

        if action == Action.Back:
            changeState(State.PlanPick)

        return

    if state == State.PlanLift:
        if action == Action.Next and future.result().success:
            changeState(State.Lift) 
            future = PandaBinPickingClient.lift() 

        # if action == Action.Next:
        #     if future:
        #         changeState(State.Lift) 
        #         future = PandaBinPickingClient.lift()
                
        #     else: 
        #         changeState(State.PlanLiftAlt)              
        #         future = PandaBinPickingClient.planLiftAlt()  

        if action == Action.Repeat:            
            future = PandaBinPickingClient.planLift()

        if action == Action.Back:
            changeState(State.Pick)

        return

    if state == State.Lift:
        if action == Action.Next and future.result().success:
            changeState(State.PlanPlace)           
            future = PandaBinPickingClient.planPlace()
            
        if action == Action.Back:
            changeState(State.PlanLift)

        return

    if state == State.PlanLiftAlt:
        if action == Action.Next and future.result().success:
            changeState(State.Lift)            
            future = PandaBinPickingClient.lift()

        if action == Action.Repeat:          
            future = PandaBinPickingClient.planLiftAlt()
            

        if action == Action.Back:
            changeState(State.PlanLift)

        return

    if state == State.LiftAlt:
        if action == Action.Next and future.result().success:
            changeState(State.PlanPlace)           
            future = PandaBinPickingClient.planPlace()
            
        if action == Action.Back:
            changeState(State.PlanLiftAlt)

        return

    if state == State.PlanPlace:
        if action == Action.Next and future.result().success:
            changeState(State.Place)           
            future = PandaBinPickingClient.place()
            
        if action == Action.Repeat:           
            future = PandaBinPickingClient.planPlace()
            

        if action == Action.Back:
            changeState(State.PlanLift)

        return

    if state == State.Place:
        if action == Action.Next and future.result().success:
            changeState(State.Capture)           
            future = PandaBinPickingClient.capture()

        if action == Action.Back:
            changeState(State.PlanPlace)

        return
