#!/usr/bin/env python3

import json
from terminator.base_termination_handler import BaseTerminationHandler

from geometry_msgs.msg import WrenchStamped, Wrench
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

import terminator.utils as t_utils


TERMINATOR_TIMER_PERIOD_NS = 1000 # nanoseconds
TERMINATOR_WRENCH_THRESH = Wrench()


class FTSTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        self.id = -1
        self.input_data_class = WrenchStamped
        self.check_rate_ns = 10E6 # 1 ms default

        self.fts_wrench = Wrench()

        # Termination condition: Magnitude threshold
        self.wrench_thresh = Wrench()
        self.wrench_thresh.force.x = 30
        self.wrench_thresh.force.y = 30
        self.wrench_thresh.force.z = 30
        self.wrench_thresh.torque.x = 2
        self.wrench_thresh.torque.y = 2
        self.wrench_thresh.torque.z = 2

    def update_config(self, cfg: TerminationConfig):
        """
        Update the termination handler config:
        - id
        - check_rate_ns
        - fts_wrench
        """
        cfg_jsons = cfg.cfg_json
        full_cfg = json.loads(cfg_jsons)
        FTS_cfg = full_cfg['FTS']
        self.id = full_cfg['id']
        self.check_rate_ns = FTS_cfg['check_rate_ns']
        self.wrench_thresh = t_utils.dict_to_wrench(FTS_cfg['threshold'])

    
    def update_input_data(self, input_signal: WrenchStamped):
        """
        Extract the wrench from the most recent FTS reading
        """
        self.fts_wrench = input_signal.wrench
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Create the termination signal and add causes based on fts_wrench magnitude and the threshold
        """
        terminate = False
        cause = "FTS termination handler caused by:\n"
        if (abs(self.fts_wrench.force.x) > self.wrench_thresh.force.x):
            terminate = True
            cause += f"Fx ({self.fts_wrench.force.x:0.2f}) exceeds threshold ({self.wrench_thresh.force.x:0.2f})"
        if (abs(self.fts_wrench.force.y) > self.wrench_thresh.force.y):
            terminate = True
            cause += f"Fy ({self.fts_wrench.force.y:0.2f}) exceeds threshold ({self.wrench_thresh.force.y:0.2f})"
        if (abs(self.fts_wrench.force.z) > self.wrench_thresh.force.z):
            terminate = True
            cause += f"Fz ({self.fts_wrench.force.z:0.2f}) exceeds threshold ({self.wrench_thresh.force.z:0.2f})"
        if (abs(self.fts_wrench.torque.x) > self.wrench_thresh.torque.x):
            terminate = True
            cause += f"Tx ({self.fts_wrench.torque.x:0.2f}) exceeds threshold ({self.wrench_thresh.torque.x:0.2f})"
        if (abs(self.fts_wrench.torque.y) > self.wrench_thresh.torque.y):
            terminate = True
            cause += f"Ty ({self.fts_wrench.torque.y:0.2f}) exceeds threshold ({self.wrench_thresh.torque.y:0.2f})"
        if (abs(self.fts_wrench.torque.z) > self.wrench_thresh.torque.z):
            terminate = True
            cause += f"Tz ({self.fts_wrench.torque.z:0.2f}) exceeds threshold ({self.wrench_thresh.torque.z:0.2f})"
        
        termination_signal = TerminationSignal()
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal