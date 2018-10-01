from gcs.msg import *
from dockingstation import *
class job(object):
    def __init__(self,target,request_giver, position):
        self.target = target
        self.request_giver = request_giver
        self.status = 1  # 1 = queued , 2 = ongoing, 3 = onhold.
        self.position = position
    def __init__(self,dock):
        pass
