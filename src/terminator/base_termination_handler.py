#!/usr/bin/env python3

import abc

from vibro_tactile_toolbox.msg import TerminationConfig, TerminationSignal

class BaseTerminationHandler(object, metaclass=abc.ABCMeta):
    """
    Abstract base class for TerminationHandler objects
    """

    def __init__(self):
        self.id = -1
        self.input_data_class = None
        self.check_rate_ns = 10E6
        self.live = False

    def update_config(self, cfg: TerminationConfig):
        """
        Callback to update the termination handler config from a json
        """
        raise NotImplementedError
    
    def update_input_data(self, input_signal):
        """
        Callback to update the input signal
        TODO: allow for multiple input signals
        """
        raise NotImplementedError
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Callback to produce the termination signal

        Note - Do not add newline characters or commas to the TerminationSignal.cause string
               This will break post processing scripts that rely on csv/text files
        """
        raise NotImplementedError