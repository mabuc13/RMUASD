from gcs.msg import *
class drone(object):
	def __init__(self, ID, Position):
		self.ID = int(ID)
		self.isFree = True
		self.position = Position
		self.goal_dock = 0
		self.path = 0
	def get_ID(self):
		return self.ID
	def get_Position(self):
		return self.position
	def set_path(self, path):
		self.path = path
	def update(self,Info):
		self.position = Info.position
	def set_Job(self, job):
		self.goal_dock = goal_dock
		self.isFree = False

	def get_info(self):
		return self.goal_dock, self.path

	def isAvailable(self):
		return self.isFree
