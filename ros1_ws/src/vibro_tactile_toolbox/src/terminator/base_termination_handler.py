#!/usr/bin/env python3

import abc

class BaseTerminationHandler(object, abc.metaclass):
    """
    Abstract termination handler class
    """
    def __init__(self):
        """
        A termination handler will have:
        1. Termination criteria
        2. Input signals
        3. Termination signal
        4. 
        """

        self.termination_criteria = None
        self.input_signals = None
        self.termination_signal = None

    def get_termination_signal(self):
        """
        
        """