def drone:
	def __init__(self, ID):
		self.ID = ID
		self.status = 0		# 0 = free , 1 = occupied
		self.start_dock
		self.goal_dock
		self.path

	def set_path(self, path):
		self.path = path

	def set_docks(self, start_dock, goal_dock):
		self.start_dock = start_dock
		self.goal_dock = goal_dock

	def get_info(self):
		return self.start_dock, self.goal_dock, self.path

	def get_status(self):
		return self.status