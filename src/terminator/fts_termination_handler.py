#!/usr/bin/env python3

import json
from terminator.base_termination_handler import BaseTerminationHandler

from geometry_msgs.msg import WrenchStamped, Wrench
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

import terminator.utils as t_utils

"""
=== FTS COMPONENT OF TERMINATION CONFIG ===
'fts':  {
    'check_rate_ns':    (int) - rate in ns to check the fts for termination
    'threshold': {      (dict) - bilateral range for non-terminal values
        'force: {       (dict) - force ramges
            'x':        (List[float]) - x range
            'y':        (List[float]) - y range 
            'z':        (List[float]) - z range
        }
        'torque: {      (dict) - torque ranges
            'x':        (List[float]) - x range
            'y':        (List[float]) - y range 
            'z':        (List[float]) - z range        
        }
    }
}
"""

class FTSTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        super().__init__()
        self.input_data_class = WrenchStamped
        self.check_rate_ns = 10E6 # 10 ms default

        self.fts_wrench = Wrench()

        # Termination condition: Bilateral threshold
        # 30 N and 2 N-m is a good safe default for preventing E-stop on the yk-gp4
        self.wrench_thresh_lo = Wrench()
        self.wrench_thresh_lo.force.x = -30
        self.wrench_thresh_lo.force.y = -30
        self.wrench_thresh_lo.force.z = -30
        self.wrench_thresh_lo.torque.x = -2
        self.wrench_thresh_lo.torque.y = -2
        self.wrench_thresh_lo.torque.z = -2

        self.wrench_thresh_hi = Wrench()
        self.wrench_thresh_hi.force.x = 30
        self.wrench_thresh_hi.force.y = 30
        self.wrench_thresh_hi.force.z = 30
        self.wrench_thresh_hi.torque.x = 2
        self.wrench_thresh_hi.torque.y = 2
        self.wrench_thresh_hi.torque.z = 2

    def update_config(self, cfg: TerminationConfig):
        """
        Update the termination handler config:
        - id
        - check_rate_ns
        - fts_wrench
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'fts' in cfg_json:
            self.live = True
            self.id = cfg_json['id']
            FTS_cfg = cfg_json['fts']
            if 'check_rate_ns' in FTS_cfg:
                self.check_rate_ns = FTS_cfg['check_rate_ns']
            if 'threshold' in FTS_cfg:
                self.wrench_thresh_lo, self.wrench_thresh_hi = t_utils.dict_to_wrench_bilateral(FTS_cfg['threshold'])
        else:
            self.live = False
    
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
        cause_header = "FTS termination handler caused by: "
        causes = []

        # Fx-
        if (self.fts_wrench.force.x < self.wrench_thresh_lo.force.x):
            causes.append(f"Fx ({self.fts_wrench.force.x:0.2f}) exceeds threshold ({self.wrench_thresh_lo.force.x:0.2f})")
        # Fx+
        if (self.fts_wrench.force.x > self.wrench_thresh_hi.force.x):
            causes.append(f"Fx ({self.fts_wrench.force.x:0.2f}) exceeds threshold ({self.wrench_thresh_hi.force.x:0.2f})")
        # Fy-
        if (self.fts_wrench.force.y < self.wrench_thresh_lo.force.y):
            causes.append(f"Fy ({self.fts_wrench.force.y:0.2f}) exceeds threshold ({self.wrench_thresh_lo.force.y:0.2f})")
        # Fy+
        if (self.fts_wrench.force.y > self.wrench_thresh_hi.force.y):
            causes.append(f"Fy ({self.fts_wrench.force.y:0.2f}) exceeds threshold ({self.wrench_thresh_hi.force.y:0.2f})")
        # Fz-
        if (self.fts_wrench.force.z < self.wrench_thresh_lo.force.z):
            causes.append(f"Fz ({self.fts_wrench.force.z:0.2f}) exceeds threshold ({self.wrench_thresh_lo.force.z:0.2f})")
        # Fz+
        if (self.fts_wrench.force.z > self.wrench_thresh_hi.force.z):
            causes.append(f"Fz ({self.fts_wrench.force.z:0.2f}) exceeds threshold ({self.wrench_thresh_hi.force.z:0.2f})")

        # Tx-
        if (self.fts_wrench.torque.x < self.wrench_thresh_lo.torque.x):
            causes.append(f"Tx ({self.fts_wrench.torque.x:0.2f}) exceeds threshold ({self.wrench_thresh_lo.torque.x:0.2f})")
        # Tx+
        if (self.fts_wrench.torque.x > self.wrench_thresh_hi.torque.x):
            causes.append(f"Tx ({self.fts_wrench.torque.x:0.2f}) exceeds threshold ({self.wrench_thresh_hi.torque.x:0.2f})")
        # Ty-
        if (self.fts_wrench.torque.y < self.wrench_thresh_lo.torque.y):
            causes.append(f"Ty ({self.fts_wrench.torque.y:0.2f}) exceeds threshold ({self.wrench_thresh_lo.torque.y:0.2f})")
        # Ty+
        if (self.fts_wrench.torque.y > self.wrench_thresh_hi.torque.y):
            causes.append(f"Ty ({self.fts_wrench.torque.y:0.2f}) exceeds threshold ({self.wrench_thresh_hi.torque.y:0.2f})")
        # Tz-
        if (self.fts_wrench.torque.z < self.wrench_thresh_lo.torque.z):
            causes.append(f"Tz ({self.fts_wrench.torque.z:0.2f}) exceeds threshold ({self.wrench_thresh_lo.torque.z:0.2f})")
        # Tz+
        if (self.fts_wrench.torque.z > self.wrench_thresh_hi.torque.z):
            causes.append(f"Tz ({self.fts_wrench.torque.z:0.2f}) exceeds threshold ({self.wrench_thresh_hi.torque.z:0.2f})")
        
        terminate = len(causes) > 0
        if terminate:
            cause = cause_header + " ".join(causes)
        else:
            cause = ''

        termination_signal = TerminationSignal()
        termination_signal.id = self.id
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal