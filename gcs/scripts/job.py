from gcs.msg import *
from dockingstation import *
class job(object):
    def __init__(self,dock):
        self.dock = dock
        self.requestGiver = dock.get_name()
        self.status = 1
        self.drone = None


        #Constants
        self.ready4takeOff = 5
        self.wait4pathplan = 4
        self.onhold = 3
        self.ongoing = 2
        self.queued = 1
    def attachDrone(self,drone):
        self.drone = drone.get_ID()
        self.wait4pathplan
