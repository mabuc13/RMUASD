from gcs.msg import *
class drone(object):
	def __init__(self, ID, Position):
		self.ID = int(ID)
		self.isFree = True
		self.start_dock = 0
		self.goal_dock = 0
		self.path = 0
	def get_ID(self):
		return self.ID

	def set_path(self, path):
		self.path = path

	def set_docks(self, start_dock, goal_dock):
		self.start_dock = start_dock
		self.goal_dock = goal_dock

	def get_info(self):
		return self.start_dock, self.goal_dock, self.path

	def isAvailable(self):
		return self.isFree
