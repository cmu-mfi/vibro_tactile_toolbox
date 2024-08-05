import sys
import os
import argparse

from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtCore import Qt, QLibraryInfo
from PyQt5 import QtGui
from PyQt5.QtGui import QPixmap, QColor
import pyqtgraph as pg

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
TERMINATION_FILENAME = 'termination_signals.txt'
FTS_OUTCOME_FILENAME = 'fts_outcomes.txt'
LEGO_OUTCOME_FILENAME = 'lego_outcomes.txt'
LEGO_DETECTIONS_DIR = 'lego_detections'

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(
    QLibraryInfo.PluginsPath
)

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

        self.vid_width = 640
        self.vid_height = 480

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
        
        # fts from .npy
        fts_data = np.load(os.path.join(self.data_path, FTS_FILENAME))
        self.t_fts = fts_data[0, :]
        self.force_data = fts_data[1:4, :]
        self.torque_data = fts_data[4:7, :]

        # joint_states from .npy
        joint_data = np.load(os.path.join(self.data_path, JOINT_FILENAME))
        self.t_joints = joint_data[0, :]
        self.joint_pos_data = joint_data[1:7, :]
        self.joint_vel_data = joint_data[7:13, :]
        self.joint_eff_data = joint_data[13:19, :]

        # cameras from .mp4
        self.side_video = os.path.join(self.data_path, SIDE_CAM_FILENAME)
        self.wrist_video = os.path.join(self.data_path, WRIST_CAM_FILENAME)

        # audio from .wav
        self.audio_data, sample_rate = sf.read(os.path.join(self.data_path, AUDIO_FILENAME))
        # Get first channel
        self.audio_data = self.audio_data[:, 0]
        self.t_audio = np.linspace(0, len(self.audio_data) / sample_rate, len(self.audio_data))

        # termination signals from .txt csv
        self.termination_data = []
        with open(os.path.join(self.data_path, TERMINATION_FILENAME), 'r') as f:
            ### FILE STRUCTURE ###
            # # commented info
            # timestamp, cause
            # timestamp, cause
            # ...
            for line in f.read().split("\n"):
                if line.startswith("#"):
                    continue
                entries = line.split(",")
                if len(entries) != 3:
                    continue

                t_trial = float(entries[0].strip())
                termination_cause = entries[2].strip()
                self.termination_data.append((t_trial, termination_cause))
        self.termination_data.sort(key=lambda x: x[0])

        # outcome signals from .txt csv
        self.outcome_data = []
        with open(os.path.join(self.data_path, LEGO_OUTCOME_FILENAME), 'r') as f:
            ### FILE STRUCTURE ###
            # # commented info
            # timestamp, lego_outcome, path/to/img_ann.png
            # timestamp, lego_outcome, path/to/img_ann.png
            # ...
            for line in f.read().split("\n"):
                if line.startswith("#"):
                    continue
                entries = line.split(",")
                if len(entries) != 4:
                    continue

                t_trial = float(entries[0].strip())
                result = entries[1].strip()
                lego_outcome = entries[2].strip()
                img_ann_file = entries[3].strip()

                img_ann = cv2.imread(os.path.join(self.data_path, img_ann_file))
                img_ann = cv2.cvtColor(img_ann, cv2.COLOR_BGR2RGB)
                height, width, channel = img_ann.shape
                bytesPerLine = 3 * width
                qimg_ann = QtGui.QImage(img_ann.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                qpixmap_ann = QPixmap.fromImage(qimg_ann).scaledToWidth(self.vid_width)
                self.outcome_data.append((t_trial, 'lego_detector', lego_outcome, qpixmap_ann))
        
        with open(os.path.join(self.data_path, FTS_OUTCOME_FILENAME), 'r') as f:
            ### FILE STRUCTURE ###
            # # commented info
            # timestamp, fts_outcome
            # timestamp, fts_outcome
            # ...
            for line in f.read().split("\n"):
                if line.startswith("#"):
                    continue
                entries = line.split(",")
                if len(entries) != 3:
                    continue

                t_trial = float(entries[0].strip())
                result = entries[1].strip()
                fts_outcome = entries[2].strip()

                self.outcome_data.append((t_trial, 'fts_detector', fts_outcome))
        
        self.outcome_data.sort(key=lambda x: x[0])


    def make_camera_group(self):
        cameraGroup = QGroupBox("Camera Group")
        cameraLayout = QVBoxLayout()

        self.side_cam = QLabel("Side Camera")
        self.wrist_cam = QLabel("Wrist Camera")

        cameraLayout.addWidget(QLabel("Side Camera"))
        cameraLayout.addWidget(self.side_cam)
        cameraLayout.addWidget(QLabel("Wrist Camera"))
        cameraLayout.addWidget(self.wrist_cam)

        grey = QPixmap(self.vid_width, self.vid_height)
        grey.fill(QColor('darkGray'))

        self.side_cam.setPixmap(grey)
        self.wrist_cam.setPixmap(grey)

        cameraGroup.setLayout(cameraLayout)
        return cameraGroup

    def make_tactile_group(self):
        tactileGroup = QGroupBox("Tactile Group")
        tactileLayout = QVBoxLayout()

        self.force_canvas = pg.PlotWidget()
        self.torque_canvas = pg.PlotWidget()
        self.audio_canvas = pg.PlotWidget()

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

        self.joint_pos_canvas = pg.PlotWidget()
        self.joint_vel_canvas = pg.PlotWidget()
        self.joint_eff_canvas = pg.PlotWidget()

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
        self.force_line = [
            self.force_canvas.plot(self.t_fts, self.force_data[0], pen=pg.mkPen(color='r', width=1), name='fx'),
            self.force_canvas.plot(self.t_fts, self.force_data[1], pen=pg.mkPen(color='g', width=1), name='fy'),
            self.force_canvas.plot(self.t_fts, self.force_data[2], pen=pg.mkPen(color='b', width=1), name='fz')
        ]
        self.force_canvas.setBackground('w')
        self.force_canvas.addLegend(offset=(0.5, 1.18))
        self.force_canvas.setRange(xRange=[0, self.t_fts[-1]])
        self.force_canvas.setLabel('left', 'N')
        self.force_canvas.getPlotItem().getAxis('left').setWidth(50)
        self.force_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.force_canvas.addItem(self.force_time)


        # FTS Torque
        self.torque_line = [
            self.torque_canvas.plot(self.t_fts, self.torque_data[0], pen=pg.mkPen(color='r', width=1), name='tx'),
            self.torque_canvas.plot(self.t_fts, self.torque_data[1], pen=pg.mkPen(color='g', width=1), name='ty'),
            self.torque_canvas.plot(self.t_fts, self.torque_data[2], pen=pg.mkPen(color='b', width=1), name='tz')
        ]
        self.torque_canvas.setBackground('w')
        self.torque_canvas.addLegend(offset=(0.5, 1.18))
        self.torque_canvas.setRange(xRange=[0, self.t_fts[-1]])
        self.torque_canvas.setLabel('left', 'N-m')
        self.torque_canvas.getPlotItem().getAxis('left').setWidth(50)
        self.torque_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.torque_canvas.addItem(self.torque_time)

        # Audio
        self.audio_line = [
            self.audio_canvas.plot(self.t_audio, self.audio_data, pen=pg.mkPen(color='r', width=1), name='EE_audio'),
        ]
        self.audio_canvas.setBackground('w')
        self.audio_canvas.addLegend(offset=(0.5, 1.18))
        self.audio_canvas.setRange(xRange=[0, self.t_fts[-1]])
        self.audio_canvas.setLabel('left', 'Amplitude')
        self.audio_canvas.getPlotItem().getAxis('left').setWidth(50)
        self.audio_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.audio_canvas.addItem(self.audio_time)

        # Joint Position
        self.joint_pos_line = [
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[0], pen=pg.mkPen(color='r', width=1), name='q1'),
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[1], pen=pg.mkPen(color='g', width=1), name='q2'),
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[2], pen=pg.mkPen(color='b', width=1), name='q3'),
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[3], pen=pg.mkPen(color='c', width=1), name='q4'),
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[4], pen=pg.mkPen(color='m', width=1), name='q5'),
            self.joint_pos_canvas.plot(self.t_joints, self.joint_pos_data[5], pen=pg.mkPen(color='y', width=1), name='q6')
        ]
        self.joint_pos_canvas.setBackground('w')
        self.joint_pos_canvas.addLegend(offset=(0.5, 1.18))
        self.joint_pos_canvas.setRange(xRange=[0, self.t_joints[-1]])
        self.joint_pos_canvas.setLabel('left', 'rad')
        self.joint_pos_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.joint_pos_canvas.addItem(self.joint_pos_time)

        # Joint Velocity
        self.joint_vel_line = [
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[0], pen=pg.mkPen(color='r', width=1), name='v1'),
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[1], pen=pg.mkPen(color='g', width=1), name='v2'),
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[2], pen=pg.mkPen(color='b', width=1), name='v3'),
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[3], pen=pg.mkPen(color='c', width=1), name='v4'),
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[4], pen=pg.mkPen(color='m', width=1), name='v5'),
            self.joint_vel_canvas.plot(self.t_joints, self.joint_vel_data[5], pen=pg.mkPen(color='y', width=1), name='v6')
        ]
        self.joint_vel_canvas.setBackground('w')
        self.joint_vel_canvas.addLegend(offset=(0.5, 1.18))
        self.joint_vel_canvas.setRange(xRange=[0, self.t_joints[-1]])
        self.joint_vel_canvas.setLabel('left', 'rad/s')
        self.joint_vel_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.joint_vel_canvas.addItem(self.joint_vel_time)

        # Joint Effort
        self.joint_eff_line = [
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[0], pen=pg.mkPen(color='r', width=1), name='u1'),
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[1], pen=pg.mkPen(color='g', width=1), name='u2'),
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[2], pen=pg.mkPen(color='b', width=1), name='u3'),
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[3], pen=pg.mkPen(color='c', width=1), name='u4'),
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[4], pen=pg.mkPen(color='m', width=1), name='u5'),
            self.joint_eff_canvas.plot(self.t_joints, self.joint_eff_data[5], pen=pg.mkPen(color='y', width=1), name='u6')
        ]
        self.joint_eff_canvas.setBackground('w')
        self.joint_eff_canvas.addLegend(offset=(0.5, 1.18))
        self.joint_eff_canvas.setRange(xRange=[0, self.t_joints[-1]])
        self.joint_eff_canvas.setLabel('left', '[effort]')
        self.joint_eff_time = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k'))
        self.joint_eff_canvas.addItem(self.joint_eff_time)

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
                self.side_pixmaps.append(QPixmap.fromImage(qImg_side).scaledToWidth(self.vid_width))

        for i_wrist in range(0, n_wrist):
            self.wrist_cap.set(cv2.CAP_PROP_POS_FRAMES, i_wrist)
            ret_wrist, frame_wrist = self.wrist_cap.read()
            if ret_wrist:
                frame_wrist = cv2.cvtColor(frame_wrist, cv2.COLOR_BGR2RGB)
                height, width, channel = frame_wrist.shape
                bytesPerLine = 3 * width
                qImg_side = QtGui.QImage(frame_wrist.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                self.wrist_pixmaps.append(QPixmap.fromImage(qImg_side).scaledToWidth(self.vid_width))

        self.side_cam.setPixmap(self.side_pixmaps[0])
        self.wrist_cam.setPixmap(self.wrist_pixmaps[0])


    def setup_slider(self):
        self.last_paused_for_outcome = 0.0
        tf = self.t_fts[-1]
        self.time_slider_max = 1000
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
        #timestep = 1/self.play_freq
        self.t_last_ticked = time.time()
        self.curr_time = self.curr_time + timestep
        new_val = self.curr_time * (self.time_slider_max / self.t_fts[-1])
        new_val_pre_mod = np.floor(new_val)
        new_val = int(new_val) % self.time_slider_max 
        if new_val < new_val_pre_mod:
            # if we've reached the end of the trial
            if (self.playing): # if playing, stop
                self.playing = False
                self.playButton.click()
                new_val = self.time_slider_max
                self.time_slider.setValue(new_val)
                return
            # else, wrap around
            self.last_paused_for_outcome = 0.0
        self.time_slider.setValue(new_val)

    def update_display(self):
        # percentage of trial time
        time_portion = self.time_slider.value() / self.time_slider_max
        t_val = time_portion * self.t_fts[-1]
        self.curr_time = t_val

        if not self.playing:
            self.last_paused_for_outcome = self.curr_time

        # Update time
        self.time_label.setText(f"{t_val:0.2f} s")

        # Update plots
        self.force_time.setPos(t_val)
        self.torque_time.setPos(t_val)
        self.audio_time.setPos(t_val)
        self.joint_pos_time.setPos(t_val)
        self.joint_vel_time.setPos(t_val)
        self.joint_eff_time.setPos(t_val)

        # Update termination signal to most recent
        prev_terminals = list(filter(lambda x: x[0] < t_val, self.termination_data))
        if len(prev_terminals) == 0:
            termination_msg = "Termination:"
        else:
            prev_terminal = prev_terminals[-1]
            termination_msg = f"Termination: Last skill step terminated at time [{prev_terminal[0]:0.2f}s] with cause [{prev_terminal[1]}]"
        self.termination_label.setText(termination_msg)

        # Update outcome to most recent
        prev_outcomes = list(filter(lambda x: x[0] < t_val, self.outcome_data))
        if len(prev_outcomes) == 0:
            prev_outcome = None
            outcome_msg = "Outcome:"
        else:
            prev_outcome = prev_outcomes[-1]
            outcome_msg = f"Outcome: Last result from {prev_outcome[1]}: {prev_outcome[2]}"
        self.outcome_label.setText(outcome_msg)

        # Update videos
        side_frame_idx = int(time_portion * self.side_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        wrist_frame_idx = int(time_portion * self.wrist_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if prev_outcome and \
           (self.last_paused_for_outcome) < prev_outcome[0] and \
           (self.curr_time) > prev_outcome[0]:
            
            self.last_paused_for_outcome = self.curr_time
            if prev_outcome[1] == 'lego_detector': self.side_cam.setPixmap(prev_outcome[-1])
            if self.play_button_pressed: self.playButton.click()
        else:
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
