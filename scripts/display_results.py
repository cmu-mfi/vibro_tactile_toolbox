import sys
import os
import argparse

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtGui import QPixmap, QColor

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import soundfile as sf

import numpy as np
import cv2

FTS_FILENAME = 'fts.npy'
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
        # [         ]   [             ]
        # [   SIDE  ]   [    FORCE    ]
        # [         ]   
        #               [             ]
        # [         ]   [    TORQUE   ]
        # [   WRIST ]
        # [         ]   [    AUDIO    ]

        # Outcome and Termination
        topLayout = QHBoxLayout()

        self.outcome_label = QLabel("Outcome: ")
        topLayout.addWidget(self.outcome_label)

        self.termination_label = QLabel("Termination: ")
        topLayout.addWidget(self.termination_label)
        
        # Camera Views
        botLeftGroup = self.make_camera_group()

        # Audio and FTS
        botRightGroup = self.make_tactile_group()

        # Time
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_label = QLabel(f"0.00s")
        self.time_label.setAlignment(Qt.AlignCenter)

        # Main layout
        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        mainLayout.addWidget(botLeftGroup, 1, 0)
        mainLayout.addWidget(botRightGroup, 1, 1)
        mainLayout.addWidget(self.time_slider, 2, 0, 1, 2)
        mainLayout.addWidget(self.time_label, 3, 0, 1, 2)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(mainLayout)

    def load_data(self):
        # Load your data here
        fts_data = np.load(os.path.join(self.data_path, FTS_FILENAME))
        self.force_data = fts_data[1:4, :]
        self.torque_data = fts_data[4:7, :]
        self.t_fts = fts_data[0, :]

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

    def setup_plots(self):
        self.force_line = self.force_canvas.axes.plot(self.t_fts, self.force_data[0], 'r-',
                                                      self.t_fts, self.force_data[1], 'g-',
                                                      self.t_fts, self.force_data[2], 'b-')
        self.force_canvas.axes.legend(['fx', 'fy', 'fz'], loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=3)
        self.force_canvas.axes.set_xlim(0, self.t_fts[-1])
        self.force_time = self.force_canvas.axes.vlines(
            0,
            self.force_canvas.axes.get_ylim()[0],
            self.force_canvas.axes.get_ylim()[1], 
            'k'
        )

        self.torque_line = self.torque_canvas.axes.plot(self.t_fts, self.torque_data[0], 'r-',
                                                        self.t_fts, self.torque_data[1], 'g-',
                                                        self.t_fts, self.torque_data[2], 'b-')
        self.torque_canvas.axes.legend(['tx', 'ty', 'tz'], loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=3)
        self.torque_canvas.axes.set_xlim(0, self.t_fts[-1])
        self.torque_time = self.torque_canvas.axes.vlines(
            0,
            self.torque_canvas.axes.get_ylim()[0],
            self.torque_canvas.axes.get_ylim()[1], 
            'k'
        )

        self.audio_line = self.audio_canvas.axes.plot(self.t_audio, self.audio_data)
        self.audio_canvas.axes.set_xlim(0, self.t_audio[-1])
        self.audio_time = self.audio_canvas.axes.vlines(
            0,
            self.audio_canvas.axes.get_ylim()[0],
            self.audio_canvas.axes.get_ylim()[1], 
            'k'
        )
        # self.fig.tight_layout()

    def setup_video_players(self):
        self.side_cap = cv2.VideoCapture(self.side_video)
        self.wrist_cap = cv2.VideoCapture(self.wrist_video)

        # Set the gui to the first frame
        self.side_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.side_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        ret_side, frame_side = self.side_cap.read()
        ret_wrist, frame_wrist = self.wrist_cap.read()

        if ret_side:
            frame_side = cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB)
            height, width, channel = frame_side.shape
            bytesPerLine = 3 * width
            qImg_side = QtGui.QImage(frame_side.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.side_cam.setPixmap(QPixmap.fromImage(qImg_side).scaledToWidth(640))

        if ret_wrist:
            frame_wrist = cv2.cvtColor(frame_wrist, cv2.COLOR_BGR2RGB)
            height, width, channel = frame_wrist.shape
            bytesPerLine = 3 * width
            qImg_wrist = QtGui.QImage(frame_wrist.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.wrist_cam.setPixmap(QPixmap.fromImage(qImg_wrist).scaledToWidth(640))


    def setup_slider(self):
        tf = self.t_fts[-1]
        self.time_slider_max = 100
        self.time_slider.setRange(0, self.time_slider_max)
        self.time_slider.valueChanged.connect(self.update_display)
        self.time_slider.setTickPosition(QSlider.TicksBelow)
        self.time_slider.setTickInterval(int(self.time_slider_max / tf))

    def update_display(self):
        # percentage of trial time
        time_portion = self.time_slider.value() / self.time_slider_max
        t_val = time_portion * self.t_fts[-1]

        # Update time
        self.time_label.setText(f"{t_val:0.2f} s")

        # Update plots
        #force_data_t = time_portion * self.t_fts[-1]
        vertecies = np.array([[t_val, self.force_canvas.axes.get_ylim()[0]], 
                              [t_val, self.force_canvas.axes.get_ylim()[1]]])
        self.force_time.set_segments([vertecies])

        #torque_data_t = time_portion * self.t_fts[-1]
        vertecies = np.array([[t_val, self.torque_canvas.axes.get_ylim()[0]], 
                              [t_val, self.torque_canvas.axes.get_ylim()[1]]])
        self.torque_time.set_segments([vertecies])

        #audio_data_t = time_portion * self.t_audio[-1]
        vertecies = np.array([[t_val, self.audio_canvas.axes.get_ylim()[0]], 
                              [t_val, self.audio_canvas.axes.get_ylim()[1]]])
        self.audio_time.set_segments([vertecies])

        self.force_canvas.draw()
        self.torque_canvas.draw()
        self.audio_canvas.draw()

        # Update videos
        side_frame_idx = int(time_portion * self.side_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.side_cap.set(cv2.CAP_PROP_POS_FRAMES, side_frame_idx)

        wrist_frame_idx = int(time_portion * self.wrist_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.wrist_cap.set(cv2.CAP_PROP_POS_FRAMES, wrist_frame_idx)

        ret_side, frame_side = self.side_cap.read()
        ret_wrist, frame_wrist = self.wrist_cap.read()

        if ret_side:
            frame_side = cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB)
            height, width, channel = frame_side.shape
            bytesPerLine = 3 * width
            qImg_side = QtGui.QImage(frame_side.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.side_cam.setPixmap(QPixmap.fromImage(qImg_side).scaledToWidth(640))

        if ret_wrist:
            frame_wrist = cv2.cvtColor(frame_wrist, cv2.COLOR_BGR2RGB)
            height, width, channel = frame_wrist.shape
            bytesPerLine = 3 * width
            qImg_wrist = QtGui.QImage(frame_wrist.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.wrist_cam.setPixmap(QPixmap.fromImage(qImg_wrist).scaledToWidth(640))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display trial data from parsed rosbags (see parse_rosbag.py)')
    parser.add_argument('--src', '-s', type=str, required=True,
                        help='data source')
    args = parser.parse_args()

    app = QApplication([])
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec_())
