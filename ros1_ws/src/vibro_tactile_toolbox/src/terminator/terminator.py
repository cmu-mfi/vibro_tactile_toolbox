from geometry_msgs.msg import WrenchStamped, Wrench

class Terminator(object):
    def __init__(self, fts_wrench_thresh: Wrench):
        self.fts_wrench_thresh = fts_wrench_thresh

    def get_termination_signal(self, fts_wrench: WrenchStamped):
        # FTS check
        if self.fts_wrench_exceeds_thresh(fts_wrench):
            return True

        return False
    
    def fts_wrench_exceeds_thresh(self, fts_wrench: WrenchStamped):
        # Basic magnitude thresholding
        if fts_wrench is None:
            return False
        fts_wrench = fts_wrench.wrench
        if (abs(fts_wrench.force.x) > self.fts_wrench_thresh.force.x):
            return True
        if (abs(fts_wrench.force.y) > self.fts_wrench_thresh.force.y):
            return True
        if (abs(fts_wrench.force.z) > self.fts_wrench_thresh.force.z):
            return True
        if (abs(fts_wrench.torque.x) > self.fts_wrench_thresh.torque.x):
            return True
        if (abs(fts_wrench.torque.y) > self.fts_wrench_thresh.torque.y):
            return True
        if (abs(fts_wrench.torque.z) > self.fts_wrench_thresh.torque.z):
            return True
        return False