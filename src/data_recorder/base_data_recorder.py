#!/usr/bin/env python3

import abc

from typing import List, Optional, Union

class BaseDataRecorder(object, metaclass=abc.ABCMeta):
    """
    Abstract base class for DataRecorder objects
    """
    def start_recording(self, file_name, params):

        raise NotImplementedError

    def stop_recording(self):

        raise NotImplementedError
