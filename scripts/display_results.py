import sys
import argparse

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtGui import QPixmap, QColor

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import numpy as np
import cv2

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
        self.width = 1000
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

        # Main layout
        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        mainLayout.addWidget(botLeftGroup, 1, 0)
        mainLayout.addWidget(botRightGroup, 1, 1)
        mainLayout.addWidget(self.time_slider, 2, 0, 1, 2)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(mainLayout)

    def load_data(self):
        # Load your data here
        self.force_data = np.random.rand(100)
        self.torque_data = np.random.rand(100)
        self.audio_data = np.random.rand(100)
        self.t_fts = np.arange(len(self.force_data))
        self.side_video = "results/name_me/side_camera.mp4"
        self.wrist_video = "results/name_me/wrist_camera.mp4"

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
        FMAX = 30
        self.force_line = self.force_canvas.axes.plot(self.t_fts, self.force_data)
        force_data_idx = int((self.time_slider.value() / 100) * len(self.t_fts))
        self.force_time = self.force_canvas.axes.vlines(force_data_idx, -FMAX, FMAX)

        TMAX = 8
        self.torque_line = self.torque_canvas.axes.plot(self.torque_data)
        torque_data_idx = int((self.time_slider.value() / 100) * len(self.t_fts))
        self.torque_time = self.torque_canvas.axes.vlines(torque_data_idx, -TMAX, TMAX)

        AMAX = max(self.audio_data)
        self.audio_line = self.audio_canvas.axes.plot(self.audio_data)
        audio_data_idx = int((self.time_slider.value() / 100) * len(self.audio_data))
        self.audio_time = self.audio_canvas.axes.vlines(audio_data_idx, -AMAX, AMAX)
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
        self.time_slider.setRange(0, 100)
        self.time_slider.valueChanged.connect(self.update_display)

    def update_display(self):
        time_step = self.time_slider.value()

        # Update plots
        FMAX = 30
        force_data_idx = int((self.time_slider.value() / 100) * len(self.t_fts))
        vertecies = np.array([[force_data_idx, -FMAX], [force_data_idx, FMAX]])
        self.force_time.set_segments([vertecies])

        TMAX = 8
        torque_data_idx = int((self.time_slider.value() / 100) * len(self.t_fts))
        vertecies = np.array([[torque_data_idx, -TMAX], [torque_data_idx, TMAX]])
        self.torque_time.set_segments([vertecies])
        
        AMAX = 100
        audio_data_idx = int((self.time_slider.value() / 100) * len(self.audio_data))
        vertecies = np.array([[audio_data_idx, -AMAX], [audio_data_idx, AMAX]])
        self.audio_time.set_segments([vertecies])

        self.force_canvas.draw()
        self.torque_canvas.draw()
        self.audio_canvas.draw()

        # Update videos
        self.side_cap.set(cv2.CAP_PROP_POS_FRAMES, time_step)
        self.wrist_cap.set(cv2.CAP_PROP_POS_FRAMES, time_step)

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
