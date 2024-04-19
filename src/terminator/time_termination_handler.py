#!/usr/bin/env python3

import rospy
import json

from vibro_tactile_toolbox.msg import TerminationConfig, TerminationSignal
from terminator.base_termination_handler import BaseTerminationHandler

class TimeTerminationHandler(BaseTerminationHandler):
    """
    Termination handler to stop on a time condition
    """
    def __init__(self):
        super().__init__()
        self.input_data_class = None
        self.check_rate_ns = 10E6 # 10ms default

        self.duration = 10.0

        self.start_time = None

    def update_config(self, cfg: TerminationConfig):
        """
        Callback to update the termination handler config from a json
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'time' in cfg_json:
            self.live = True
            self.id = cfg_json['id']
            time_cfg = cfg_json['time']
            if 'duration' in time_cfg:
                self.duration = time_cfg['duration']
            # Always reset the time if a time termination is requested
            self.start_time = rospy.get_time()
        else:
            self.live = False
    
    def update_input_data(self, input_signal):
        """
        Callback to update the input signal
        """
        # No input signal, we'll keep track of time internally since there's no topic to subscribe to
        pass
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Callback to produce the termination signal
        """
        termination_signal = TerminationSignal()
        curr_time = rospy.get_time()
        terminate = (self.start_time is not None) and (curr_time - self.start_time) > self.duration
        cause = f"Time termination handler caused by:\n"
        if terminate:
            cause += f"Command execution time after {(curr_time - self.start_time):0.2f}s with time duration {self.duration:0.2f}s"
        
        termination_signal.id = self.id
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal