import sys
import os
import argparse

from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtGui import QPixmap, QColor

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import soundfile as sf

import numpy as np
import cv2

import time

FTS_FILENAME = 'fts.npy'
JOINT_FILENAME = 'joint_states.npy'
AUDIO_FILENAME = 'audio.wav'
SIDE_CAM_FILENAME = 'side_camera.mp4'
WRIST_CAM_FILENAME = 'wrist_camera.mp4'

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()

        self.data_path = args.src

        trial_name = self.data_path.split('/')[-1]
        self.title = f"Viewing Trial: {trial_name}"

        self.curr_time = 0 # used for the time slider & play button
        self.t_last_ticked = None # used for play button

        self.top = 100
        self.left = 100
        self.width = 1600
        self.height = 1000

        self.setup_ui()
        self.load_data()
        self.setup_plots()
        self.setup_video_players()
        self.setup_slider()

    def setup_ui(self):
        # Setup the window
        #self.setWindowIcon(QtGui.QIcon("icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)

        # [ OUTCOME ]   [ TERMINATION ]
        # [         ]   [             ] [            ]
        # [   SIDE  ]   [    FORCE    ] [   POS      ]
        # [         ]   
        #               [             ] [            ]
        # [         ]   [    TORQUE   ] [   VEL      ]
        # [   WRIST ]
        # [         ]   [    AUDIO    ] [   EFFORT   ]

        # Outcome and Termination
        topLayout = QHBoxLayout()

        self.outcome_label = QLabel("Outcome: ")
        topLayout.addWidget(self.outcome_label)

        self.termination_label = QLabel("Termination: ")
        topLayout.addWidget(self.termination_label)
        
        # Camera Views
        botLeftGroup = self.make_camera_group()

        # Audio and FTS
        botMidGroup = self.make_tactile_group()

        # Joint States
        botRightGroup = self.make_joint_group()

        # Time
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_label = QLabel(f"0.00s")
        self.time_label.setAlignment(Qt.AlignCenter)

        self.playTimer = QtCore.QTimer()
        self.play_freq = 15 # Hz
        self.playTimer.setInterval(int(1000/self.play_freq))
        self.playTimer.timeout.connect(self.playTick)
        self.playButton = QPushButton()
        self.playButton.setText("Play")
        self.playButton.clicked.connect(self.play_button_pressed)
        self.playing = False

        # Main layout
        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 3)
        mainLayout.addWidget(botLeftGroup, 1, 0)
        mainLayout.addWidget(botMidGroup, 1, 1)
        mainLayout.addWidget(botRightGroup, 1, 2)
        mainLayout.addWidget(self.time_slider, 2, 0, 1, 3)
        mainLayout.addWidget(self.time_label, 3, 1, 1, 2)
        mainLayout.addWidget(self.playButton, 3, 0)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(mainLayout)

    def load_data(self):
        # Load your data here
        fts_data = np.load(os.path.join(self.data_path, FTS_FILENAME))
        self.t_fts = fts_data[0, :]
        self.force_data = fts_data[1:4, :]
        self.torque_data = fts_data[4:7, :]

        joint_data = np.load(os.path.join(self.data_path, JOINT_FILENAME))
        self.t_joints = joint_data[0, :]
        self.joint_pos_data = joint_data[1:7, :]
        self.joint_vel_data = joint_data[7:13, :]
        self.joint_eff_data = joint_data[13:19, :]

        self.side_video = os.path.join(self.data_path, SIDE_CAM_FILENAME)
        self.wrist_video = os.path.join(self.data_path, WRIST_CAM_FILENAME)

        self.audio_data, sample_rate = sf.read(os.path.join(self.data_path, AUDIO_FILENAME))
        # Get first channel
        self.audio_data = self.audio_data[:, 0]
        self.t_audio = np.linspace(0, len(self.audio_data) / sample_rate, len(self.audio_data))


    def make_camera_group(self):
        cameraGroup = QGroupBox("Camera Group")
        cameraLayout = QVBoxLayout()

        self.side_cam = QLabel("Side Camera")
        self.wrist_cam = QLabel("Wrist Camera")

        cameraLayout.addWidget(QLabel("Side Camera"))
        cameraLayout.addWidget(self.side_cam)
        cameraLayout.addWidget(QLabel("Wrist Camera"))
        cameraLayout.addWidget(self.wrist_cam)

        grey = QPixmap(640, 480)
        grey.fill(QColor('darkGray'))

        self.side_cam.setPixmap(grey)
        self.wrist_cam.setPixmap(grey)

        cameraGroup.setLayout(cameraLayout)
        return cameraGroup

    def make_tactile_group(self):
        tactileGroup = QGroupBox("Tactile Group")
        tactileLayout = QVBoxLayout()

        self.force_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.torque_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.audio_canvas = MplCanvas(self, width=5, height=4, dpi=100)

        tactileLayout.addWidget(QLabel("Force"))
        tactileLayout.addWidget(self.force_canvas)
        tactileLayout.addWidget(QLabel("Torque"))
        tactileLayout.addWidget(self.torque_canvas)
        tactileLayout.addWidget(QLabel("Audio"))
        tactileLayout.addWidget(self.audio_canvas)

        tactileGroup.setLayout(tactileLayout)
        return tactileGroup

    def make_joint_group(self):
        jointGroup = QGroupBox("Tactile Group")
        jointLayout = QVBoxLayout()

        self.joint_pos_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.joint_vel_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.joint_eff_canvas = MplCanvas(self, width=5, height=4, dpi=100)

        jointLayout.addWidget(QLabel("Joint Position"))
        jointLayout.addWidget(self.joint_pos_canvas)
        jointLayout.addWidget(QLabel("Joint Velocity"))
        jointLayout.addWidget(self.joint_vel_canvas)
        jointLayout.addWidget(QLabel("Joint Effort"))
        jointLayout.addWidget(self.joint_eff_canvas)

        jointGroup.setLayout(jointLayout)
        return jointGroup

    def setup_plots(self):
        # FTS Force
        self.force_line = self.force_canvas.axes.plot(self.t_fts, self.force_data[0], 'r-',
                                                      self.t_fts, self.force_data[1], 'g-',
                                                      self.t_fts, self.force_data[2], 'b-')
        self.force_canvas.axes.legend(['fx', 'fy', 'fz'], loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=3)
        self.force_canvas.axes.set_xlim(0, self.t_fts[-1])
        self.force_canvas.axes.set_ylabel('N')
        self.force_time = self.force_canvas.axes.vlines(
            0,
            self.force_canvas.axes.get_ylim()[0],
            self.force_canvas.axes.get_ylim()[1], 
            'k'
        )

        # FTS Torque
        self.torque_line = self.torque_canvas.axes.plot(self.t_fts, self.torque_data[0], 'r-',
                                                        self.t_fts, self.torque_data[1], 'g-',
                                                        self.t_fts, self.torque_data[2], 'b-')
        self.torque_canvas.axes.legend(['tx', 'ty', 'tz'], loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=3)
        self.torque_canvas.axes.set_xlim(0, self.t_fts[-1])
        self.torque_canvas.axes.set_ylabel('N-m')
        self.torque_time = self.torque_canvas.axes.vlines(
            0,
            self.torque_canvas.axes.get_ylim()[0],
            self.torque_canvas.axes.get_ylim()[1], 
            'k'
        )

        # Audio
        self.audio_line = self.audio_canvas.axes.plot(self.t_audio, self.audio_data)
        self.audio_canvas.axes.set_xlim(0, self.t_audio[-1])
        self.audio_time = self.audio_canvas.axes.vlines(
            0,
            self.audio_canvas.axes.get_ylim()[0],
            self.audio_canvas.axes.get_ylim()[1], 
            'k'
        )

        # Joint Position
        self.joint_pos_line = self.joint_pos_canvas.axes.plot(
            self.t_joints, self.joint_pos_data[0], 'r-',
            self.t_joints, self.joint_pos_data[1], 'g-',
            self.t_joints, self.joint_pos_data[2], 'b-',
            self.t_joints, self.joint_pos_data[3], 'c-',
            self.t_joints, self.joint_pos_data[4], 'm-',
            self.t_joints, self.joint_pos_data[5], 'y-'
        )
        self.joint_pos_canvas.axes.legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'], loc='upper center', 
                                          bbox_to_anchor=(0.5, 1.18), ncol=6)
        self.joint_pos_canvas.axes.set_xlim(0, self.t_joints[-1])
        self.joint_pos_time = self.joint_pos_canvas.axes.vlines(
            0,
            self.joint_pos_canvas.axes.get_ylim()[0],
            self.joint_pos_canvas.axes.get_ylim()[1], 
            'k'
        )

        # Joint Velocity
        self.joint_vel_line = self.joint_vel_canvas.axes.plot(
            self.t_joints, self.joint_vel_data[0], 'r-',
            self.t_joints, self.joint_vel_data[1], 'g-',
            self.t_joints, self.joint_vel_data[2], 'b-',
            self.t_joints, self.joint_vel_data[3], 'c-',
            self.t_joints, self.joint_vel_data[4], 'm-',
            self.t_joints, self.joint_vel_data[5], 'y-'
        )
        self.joint_vel_canvas.axes.legend(['v1', 'v2', 'v3', 'v4', 'v5', 'v6'], loc='upper center', 
                                          bbox_to_anchor=(0.5, 1.18), ncol=6)
        self.joint_vel_canvas.axes.set_xlim(0, self.t_joints[-1])
        self.joint_vel_time = self.joint_vel_canvas.axes.vlines(
            0,
            self.joint_vel_canvas.axes.get_ylim()[0],
            self.joint_vel_canvas.axes.get_ylim()[1], 
            'k'
        )

        # Joint Effort
        self.joint_eff_line = self.joint_eff_canvas.axes.plot(
            self.t_joints, self.joint_eff_data[0], 'r-',
            self.t_joints, self.joint_eff_data[1], 'g-',
            self.t_joints, self.joint_eff_data[2], 'b-',
            self.t_joints, self.joint_eff_data[3], 'c-',
            self.t_joints, self.joint_eff_data[4], 'm-',
            self.t_joints, self.joint_eff_data[5], 'y-'
        )
        self.joint_eff_canvas.axes.legend(['u1', 'u2', 'u3', 'u4', 'u5', 'u6'], loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=6)
        self.joint_eff_canvas.axes.set_xlim(0, self.t_joints[-1])
        self.joint_eff_time = self.joint_eff_canvas.axes.vlines(
            0,
            self.joint_eff_canvas.axes.get_ylim()[0],
            self.joint_eff_canvas.axes.get_ylim()[1], 
            'k'
        )

    def setup_video_players(self):
        print(f"Converting all video frames to pixmaps...")
        self.side_cap = cv2.VideoCapture(self.side_video)
        self.wrist_cap = cv2.VideoCapture(self.wrist_video)

        self.side_pixmaps = []
        self.wrist_pixmaps = []

        n_side = int(self.side_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        n_wrist = int(self.wrist_cap.get(cv2.CAP_PROP_FRAME_COUNT))

        for i_side in range(0, n_side):
            self.side_cap.set(cv2.CAP_PROP_POS_FRAMES, i_side)
            ret_side, frame_side = self.side_cap.read()
            if ret_side:
                frame_side = cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB)
                height, width, channel = frame_side.shape
                bytesPerLine = 3 * width
                qImg_side = QtGui.QImage(frame_side.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                self.side_pixmaps.append(QPixmap.fromImage(qImg_side).scaledToWidth(640))

        for i_wrist in range(0, n_wrist):
            self.wrist_cap.set(cv2.CAP_PROP_POS_FRAMES, i_wrist)
            ret_wrist, frame_wrist = self.wrist_cap.read()
            if ret_wrist:
                frame_wrist = cv2.cvtColor(frame_wrist, cv2.COLOR_BGR2RGB)
                height, width, channel = frame_wrist.shape
                bytesPerLine = 3 * width
                qImg_side = QtGui.QImage(frame_wrist.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                self.wrist_pixmaps.append(QPixmap.fromImage(qImg_side).scaledToWidth(640))

        self.side_cam.setPixmap(self.side_pixmaps[0])
        self.wrist_cam.setPixmap(self.wrist_pixmaps[0])


    def setup_slider(self):
        tf = self.t_fts[-1]
        self.time_slider_max = 200
        self.time_slider.setRange(0, self.time_slider_max)
        self.time_slider.valueChanged.connect(self.update_display)
        self.time_slider.setTickPosition(QSlider.TicksBelow)
        self.time_slider.setTickInterval(int(self.time_slider_max / tf))

    def play_button_pressed(self):
        if not self.playing:
            self.playing = True
            self.t_last_ticked = time.time()
            self.playTimer.start()
        else:
            self.playing = False
            self.playTimer.stop()

    def playTick(self):
        timestep = time.time() - self.t_last_ticked
        self.t_last_ticked = time.time()
        self.curr_time = self.curr_time + timestep
        new_val = self.curr_time * (self.time_slider_max / self.t_fts[-1])
        new_val = int(new_val) % self.time_slider_max 
        self.time_slider.setValue(new_val)

    def update_display(self):
        # percentage of trial time
        time_portion = self.time_slider.value() / self.time_slider_max
        t_val = time_portion * self.t_fts[-1]
        self.curr_time = t_val

        # Update time
        self.time_label.setText(f"{t_val:0.2f} s")

        # Update plots
        #force_data_t = time_portion * self.t_fts[-1]
        vertices = np.array([[t_val, self.force_canvas.axes.get_ylim()[0]], 
                              [t_val, self.force_canvas.axes.get_ylim()[1]]])
        self.force_time.set_segments([vertices])

        #torque_data_t = time_portion * self.t_fts[-1]
        vertices = np.array([[t_val, self.torque_canvas.axes.get_ylim()[0]], 
                              [t_val, self.torque_canvas.axes.get_ylim()[1]]])
        self.torque_time.set_segments([vertices])

        #audio_data_t = time_portion * self.t_audio[-1]
        vertices = np.array([[t_val, self.audio_canvas.axes.get_ylim()[0]], 
                              [t_val, self.audio_canvas.axes.get_ylim()[1]]])
        self.audio_time.set_segments([vertices])

        vertices = np.array([[t_val, self.joint_pos_canvas.axes.get_ylim()[0]],
                             [t_val, self.joint_pos_canvas.axes.get_ylim()[1]]])
        self.joint_pos_time.set_segments([vertices])

        vertices = np.array([[t_val, self.joint_vel_canvas.axes.get_ylim()[0]],
                             [t_val, self.joint_vel_canvas.axes.get_ylim()[1]]])
        self.joint_vel_time.set_segments([vertices])

        vertices = np.array([[t_val, self.joint_eff_canvas.axes.get_ylim()[0]],
                             [t_val, self.joint_eff_canvas.axes.get_ylim()[1]]])
        self.joint_eff_time.set_segments([vertices])

        self.force_canvas.draw()
        self.torque_canvas.draw()
        self.audio_canvas.draw()
        self.joint_pos_canvas.draw()
        self.joint_vel_canvas.draw()
        self.joint_eff_canvas.draw()

        # Update videos
        side_frame_idx = int(time_portion * self.side_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        wrist_frame_idx = int(time_portion * self.wrist_cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.side_cam.setPixmap(self.side_pixmaps[side_frame_idx])
        self.wrist_cam.setPixmap(self.wrist_pixmaps[wrist_frame_idx])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display trial data from parsed rosbags (see parse_rosbag.py)')
    parser.add_argument('--src', '-s', type=str, required=True,
                        help='data source')
    args = parser.parse_args()

    app = QApplication([])
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec_())
