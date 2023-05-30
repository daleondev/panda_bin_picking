from state_machine import *
from settings import *

import rviz

#python_qt_binding
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

bg_0 = "#303030"
bg_1 = "#808080"
bg_2 = "#737373"

fg_0 = "#d0d0d0"
fg_1 = "#d0d0d0"
fg_2 = "#b2b2b2"

def createOutput():
    layout_output = QVBoxLayout()

    spacer = QSpacerItem(20, 5, QSizePolicy.Minimum, QSizePolicy.Minimum)

    text_output = QTextBrowser()
    text_output.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)    
    text_output.setMaximumWidth(200)
    text_output.setFontPointSize(10)
    text_output.setStyleSheet(  "background: " + bg_2 + ";"+ 
                                "color: " + fg_0 + ";"+
                                "border-radius: 0px;")

    layout_output.addSpacerItem(spacer)
    layout_output.addWidget(text_output)
    layout_output.addSpacerItem(spacer)
    layout_output.setAlignment(text_output, Qt.AlignHCenter)

    output.append(text_output)

    frame_output = QFrame()
    frame_output.setLayout(layout_output)
    frame_output.setStyleSheet("QFrame {background: "+bg_1+"; border-bottom-left-radius: 12px; border-bottom-right-radius: 12px; border-top-right-radius: 12px;}")

    return frame_output

def createSettings():
    layout_settings = QVBoxLayout()

    spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

    lines = []
    for i in range(0, 4):
        temp = QWidget()
        temp.setFixedHeight(2)
        temp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        temp.setStyleSheet("background: " + bg_2 + ";")
        lines.append(temp)
    
    layout_hand         = QVBoxLayout()
    layout_grasp        = QVBoxLayout()
    layout_generation   = QVBoxLayout()
    layout_filter       = QVBoxLayout()

    label_hand          = QLabel("Hand Geometry [cm]")
    label_grasp         = QLabel("Grasp Descriptor [cm]")
    label_generation    = QLabel("Candidate Generation")
    label_filter        = QLabel("Candidate Filtering")

    layout_hand_finger      = QHBoxLayout()
    layout_hand_diameter    = QHBoxLayout()
    layout_hand_depth       = QHBoxLayout()
    layout_hand_height      = QHBoxLayout()

    label_hand_finger       = QLabel("Finger Width")
    label_hand_diameter     = QLabel("Outer Diameter")
    label_hand_depth        = QLabel("Hand Depth")
    label_hand_height       = QLabel("Hand Height")

    input_hand_finger       = QLineEdit()   
    input_hand_diameter     = QLineEdit()
    input_hand_depth        = QLineEdit()
    input_hand_height       = QLineEdit()

    input_hand_finger.setFixedWidth(80)
    input_hand_diameter.setFixedWidth(80)
    input_hand_depth.setFixedWidth(80)
    input_hand_height.setFixedWidth(80)

    layout_hand_finger.addWidget(label_hand_finger)
    layout_hand_finger.addWidget(input_hand_finger)
    layout_hand_diameter.addWidget(label_hand_diameter)
    layout_hand_diameter.addWidget(input_hand_diameter)
    layout_hand_depth.addWidget(label_hand_depth)
    layout_hand_depth.addWidget(input_hand_depth)
    layout_hand_height.addWidget(label_hand_height)
    layout_hand_height.addWidget(input_hand_height)

    layout_hand.addWidget(label_hand)
    layout_hand.setAlignment(label_hand, Qt.AlignHCenter)
    layout_hand.addLayout(layout_hand_finger)
    layout_hand.addLayout(layout_hand_diameter)
    layout_hand.addLayout(layout_hand_depth)
    layout_hand.addLayout(layout_hand_height)

    layout_grasp_width  = QHBoxLayout()
    layout_grasp_depth  = QHBoxLayout()
    layout_grasp_height = QHBoxLayout()

    label_grasp_width   = QLabel("Volume Width")
    label_grasp_depth   = QLabel("Volume Depth")
    label_grasp_height  = QLabel("Volume Height")

    input_grasp_width   = QLineEdit()
    input_grasp_depth   = QLineEdit()
    input_grasp_height  = QLineEdit()

    input_grasp_width.setFixedWidth(80)
    input_grasp_depth.setFixedWidth(80)
    input_grasp_height.setFixedWidth(80)

    layout_grasp_width.addWidget(label_grasp_width)
    layout_grasp_width.addWidget(input_grasp_width)
    layout_grasp_depth.addWidget(label_grasp_depth)
    layout_grasp_depth.addWidget(input_grasp_depth)
    layout_grasp_height.addWidget(label_grasp_height)
    layout_grasp_height.addWidget(input_grasp_height)

    layout_grasp.addWidget(label_grasp)
    layout_grasp.setAlignment(label_grasp, Qt.AlignHCenter)
    layout_grasp.addLayout(layout_grasp_width)
    layout_grasp.addLayout(layout_grasp_depth)
    layout_grasp.addLayout(layout_grasp_height)

    layout_generation_selected      = QHBoxLayout()
    layout_generation_sampling      = QHBoxLayout()
    layout_generation_orientations  = QHBoxLayout()
    layout_generation_placements    = QHBoxLayout()
    layout_generation_deepen        = QHBoxLayout()

    label_generation_selected       = QLabel("Num Selected")
    label_generation_sampling       = QLabel("Num Samples")
    label_generation_orientations   = QLabel("Num Orientations")
    label_generation_placements     = QLabel("Num Positions")
    label_generation_deepen         = QLabel("Deepen Hand")

    input_generation_selected       = QLineEdit()
    input_generation_sampling       = QLineEdit()
    input_generation_orientations   = QLineEdit()
    input_generation_placements     = QLineEdit()
    input_generation_deepen         = QLineEdit()

    input_generation_selected.setFixedWidth(80)
    input_generation_sampling.setFixedWidth(80)
    input_generation_orientations.setFixedWidth(80)
    input_generation_placements.setFixedWidth(80)
    input_generation_deepen.setFixedWidth(80)

    layout_generation_selected.addWidget(label_generation_selected)
    layout_generation_selected.addWidget(input_generation_selected)
    layout_generation_sampling.addWidget(label_generation_sampling)
    layout_generation_sampling.addWidget(input_generation_sampling)
    layout_generation_orientations.addWidget(label_generation_orientations)
    layout_generation_orientations.addWidget(input_generation_orientations)
    layout_generation_placements.addWidget(label_generation_placements)
    layout_generation_placements.addWidget(input_generation_placements)
    layout_generation_deepen.addWidget(label_generation_deepen)
    layout_generation_deepen.addWidget(input_generation_deepen)

    layout_generation.addWidget(label_generation)
    layout_generation.setAlignment(label_generation, Qt.AlignHCenter)
    layout_generation.addLayout(layout_generation_selected)
    layout_generation.addLayout(layout_generation_sampling)
    layout_generation.addLayout(layout_generation_orientations)
    layout_generation.addLayout(layout_generation_placements)
    layout_generation.addLayout(layout_generation_deepen)

    layout_filter_workspace = QHBoxLayout()
    layout_filter_enable    = QHBoxLayout()
    layout_filter_direction = QHBoxLayout()
    layout_filter_thresh    = QHBoxLayout()

    label_filter_workspace  = QLabel("Workspace")
    label_filter_enable     = QLabel("Enable Filtering")
    label_filter_direction  = QLabel("Filter Direction")
    label_filter_thresh     = QLabel("Angle Tresh [rad]")

    input_filter_workspace  = QLineEdit()
    input_filter_enable     = QLineEdit()
    input_filter_direction  = QLineEdit()
    input_filter_thresh     = QLineEdit()

    input_filter_enable.setFixedWidth(80)
    input_filter_direction.setFixedWidth(80)
    input_filter_thresh.setFixedWidth(80)

    layout_filter_workspace.addWidget(label_filter_workspace)
    layout_filter_workspace.addWidget(input_filter_workspace)
    layout_filter_enable.addWidget(label_filter_enable)
    layout_filter_enable.addWidget(input_filter_enable)
    layout_filter_direction.addWidget(label_filter_direction)
    layout_filter_direction.addWidget(input_filter_direction)
    layout_filter_thresh.addWidget(label_filter_thresh)
    layout_filter_thresh.addWidget(input_filter_thresh)

    layout_filter.addWidget(label_filter)
    layout_filter.setAlignment(label_filter, Qt.AlignHCenter)
    layout_filter.addLayout(layout_filter_workspace)
    layout_filter.addLayout(layout_filter_enable)
    layout_filter.addLayout(layout_filter_direction)
    layout_filter.addLayout(layout_filter_thresh)

    button_confirm = QPushButton("Confirm")
    button_confirm.clicked.connect(write_settings) 
    button_confirm.setStyleSheet("background: " + bg_1 + ";")

    layout_settings.addSpacerItem(spacer)
    layout_settings.addLayout(layout_hand)
    layout_settings.addWidget(lines[0])
    layout_settings.addSpacerItem(spacer)
    layout_settings.addLayout(layout_grasp)
    layout_settings.addWidget(lines[1])
    layout_settings.addSpacerItem(spacer)
    layout_settings.addLayout(layout_generation)
    layout_settings.addWidget(lines[2])
    layout_settings.addSpacerItem(spacer)
    layout_settings.addLayout(layout_filter)
    layout_settings.addSpacerItem(spacer)
    layout_settings.addWidget(button_confirm)

    inputs.append(input_hand_finger)
    inputs.append(input_hand_diameter)
    inputs.append(input_hand_depth)
    inputs.append(input_hand_height)

    inputs.append(input_grasp_width)
    inputs.append(input_grasp_depth)
    inputs.append(input_grasp_height)

    inputs.append(input_generation_selected)
    inputs.append(input_generation_sampling)
    inputs.append(input_generation_orientations)
    inputs.append(input_generation_placements)
    inputs.append(input_generation_deepen)

    inputs.append(input_filter_workspace)
    inputs.append(input_filter_enable)
    inputs.append(input_filter_direction)
    inputs.append(input_filter_thresh)   

    for input in inputs:
        if input == input_filter_workspace:
            continue

        input.setValidator(QDoubleValidator(0, 100, -1))

    frame_settings = QFrame()
    frame_settings.setLayout(layout_settings)
    frame_settings.setStyleSheet(   "QFrame {background: " + bg_1 + "; border-bottom-left-radius: 12px; border-bottom-right-radius: 12px; border-top-right-radius: 12px;}"+
                                    "QLineEdit {background: " + bg_2 + "; color: " + fg_0 + "}")

    return frame_settings

def createMenu():
    layout_menu = QVBoxLayout()  
    widget_tab = QTabWidget()

    widget_tab.addTab(createOutput(), "Output")   
    widget_tab.addTab(createSettings(), "Settings") 
    widget_tab.setStyleSheet("""
QTabWidget::pane {
    border: 0px;
}
QTabBar::tab {
    background-color: """+bg_2+""";
    min-width: 70px;
    padding-top : 4px;
    padding-bottom : 4px;
    border-right: 2px solid """+bg_0+""";
}
QTabBar::tab:selected, QTabBar::tab:hover {
    background-color: """+bg_1+""";
}
QTabBar::tab:!selected {
    border-bottom: 2px solid """+bg_0+""";;
}""")

    layout_menu.addWidget(widget_tab)

    frame_menu = QFrame()
    frame_menu.setLayout(layout_menu)
    frame_menu.setMinimumWidth(200)
    frame_menu.setMaximumWidth(250)
    frame_menu.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)   

    return frame_menu

def createVisualization():
    layout_vis      = QVBoxLayout()
    layout_inner    = QHBoxLayout()

    icon_repeat = QIcon(os.path.join(os.path.abspath(os.path.dirname(__file__)), "icons/repeat.png"))
    icon_back   = QIcon(os.path.join(os.path.abspath(os.path.dirname(__file__)), "icons/back.png"))
    icon_resume = QIcon(os.path.join(os.path.abspath(os.path.dirname(__file__)), "icons/resume.png"))
    #icon_stop   = QIcon(os.path.join(os.path.abspath(os.path.dirname(__file__)), "icons/stop.png"))
    icon_next   = QIcon(os.path.join(os.path.abspath(os.path.dirname(__file__)), "icons/next.png"))

    spacer = [  QSpacerItem(20, 10, QSizePolicy.Minimum, QSizePolicy.Minimum),
                QSpacerItem(20, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)   ]

    config = rviz.Config()
    rviz.YamlConfigReader().readFile( config, os.path.join(os.path.abspath(os.path.dirname(__file__)), "config.rviz") )
    frame_rviz = rviz.VisualizationFrame()
    frame_rviz.setSplashPath("")
    frame_rviz.initialize()
    frame_rviz.load(config)
    frame_rviz.setMenuBar(None)
    frame_rviz.setStatusBar(None)
    frame_rviz.setHideButtonVisibility(False)
    frame_rviz.setStyleSheet("background: " + bg_1 + ";")

    button_repeat = QPushButton()
    button_repeat.setCursor(QCursor(Qt.PointingHandCursor))
    button_repeat.clicked.connect(lambda: stateMachine(Action.Repeat))
    button_repeat.setIcon(icon_repeat)
    button_repeat.setEnabled(False)
    button_repeat.setIconSize(QSize(20, 20))
    button_repeat.setStyleSheet("background: " + bg_1 + ";")
    
    button_back = QPushButton()
    button_back.setCursor(QCursor(Qt.PointingHandCursor))  
    button_back.clicked.connect(lambda: stateMachine(Action.Back)) 
    button_back.setIcon(icon_back)
    button_back.setEnabled(False)
    button_back.setIconSize(QSize(20, 20))
    button_back.setStyleSheet("background: " + bg_1 + ";")

    button_auto = QPushButton()
    button_auto.setCursor(QCursor(Qt.PointingHandCursor))
    button_auto.setIcon(icon_resume)
    button_auto.setEnabled(False)
    button_auto.setStyleSheet("background: " + bg_1 + ";")

    button_next = QPushButton()
    button_next.setCursor(QCursor(Qt.PointingHandCursor))
    button_next.clicked.connect(lambda: stateMachine(Action.Next))
    button_next.setIcon(icon_next)
    button_next.setIconSize(QSize(20, 20))
    button_next.setStyleSheet("background: " + bg_1 + ";")

    label_prev = QLabel("Previous State: ")
    label_prev.setStyleSheet(   "background: " + bg_1 + ";"+ 
                                "color: " + fg_1 + ";")
    text_prev = QLineEdit("-")
    text_prev.setMaximumWidth(100)
    text_prev.setReadOnly(True)
    text_prev.setStyleSheet(    "background: " + bg_2 + ";"+ 
                                "color: " + fg_0 + ";") 

    label_curr = QLabel("Current State: ")
    label_curr.setStyleSheet(   "background: " + bg_1 + ";"+ 
                                "color: " + fg_1 + ";")
    text_curr = QLineEdit("Home")
    text_curr.setMaximumWidth(100)
    text_curr.setReadOnly(True)
    text_curr.setStyleSheet(    "background: " + bg_2 + ";"+ 
                                "color: " + fg_0 + ";") 

    label_next = QLabel("Next State: ")
    label_next.setStyleSheet(   "background: " + bg_1 + ";"+ 
                                "color: " + fg_1 + ";")
    text_next = QLineEdit("Capture")
    text_next.setMaximumWidth(100)
    text_next.setReadOnly(True)
    text_next.setStyleSheet(    "background: " + bg_2 + ";"+ 
                                "color: " + fg_0 + ";") 

    layout_inner = QHBoxLayout() 
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(button_back)
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(button_auto)
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(button_next)
    layout_inner.addSpacerItem(spacer[1])
    layout_inner.addWidget(label_prev)
    layout_inner.addWidget(text_prev)
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(label_curr)
    layout_inner.addWidget(text_curr)
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(label_next)
    layout_inner.addWidget(text_next)
    layout_inner.addSpacerItem(spacer[0])
    layout_inner.addWidget(button_repeat)
    layout_inner.addSpacerItem(spacer[0])
    
    layout_vis.addWidget(frame_rviz)
    layout_vis.addLayout(layout_inner)

    buttons.append(button_repeat)
    buttons.append(button_back)
    buttons.append(button_auto)
    buttons.append(button_next)

    out_state.append(text_prev)
    out_state.append(text_curr)
    out_state.append(text_next)

    frame_viz = QFrame()
    frame_viz.setLayout(layout_vis)
    frame_viz.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    frame_viz.setStyleSheet("QFrame {background: "+bg_1+"; border-radius: 12px;} "+
                            "QFrame {margin-bottom: 8px;}")

    return frame_viz

def createCentalWidget(parent):
    central_widget = QWidget(parent)
    layout_main = QVBoxLayout(central_widget)
    layout_content = QHBoxLayout()

    spacer = QSpacerItem(5, 20, QSizePolicy.Minimum, QSizePolicy.Minimum)

    layout_content.addWidget(createMenu())
    layout_content.addSpacerItem(spacer)
    layout_content.addWidget(createVisualization())
    layout_content.addSpacerItem(spacer)

    layout_main.addLayout(layout_content)
    layout_main.addSpacerItem(spacer)

    central_widget.setLayout(layout_main)
    central_widget.setStyleSheet(   "QWidget {background: " + bg_0 + ";"+
                                    "color: " + fg_1 + ";} ")

    return central_widget

def CreateMenuBar(win):
    menu_bar = QMenuBar()

    # action_reset = QAction("&Reset", win)
    # action_reset.setShortcut("Ctrl+R")
    # action_reset.setStatusTip('Reset to Home-State')
    # action_reset.triggered.connect(lambda: changeState(Action.Reset))

    action_quit = QAction("&Quit", win)
    action_quit.setShortcut("Ctrl+C")
    action_quit.setStatusTip('Quit the App')
    action_quit.triggered.connect(lambda: [PandaBinPickingClient.shutDown(), QApplication.exit()])
    
    menu_options = QMenu("&Options", win)
    # menu_options.addAction(action_reset)
    menu_options.addAction(action_quit)

    menu_bar.addMenu(menu_options)   
    menu_bar.setStyleSheet( "background: " + bg_0 + ";"+ 
                            "color: " + fg_1 + ";")     


    return menu_bar

def createWindow():
    window = QMainWindow()
    window.resize(1280, 840)

    window.setMenuBar(CreateMenuBar(window))
    window.setCentralWidget(createCentalWidget(window))

    load_settings()
    
    return window