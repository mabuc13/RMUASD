from gcs.msg import GPS

class dockingstation(object):
	def __init__(self,Name,latitude,longitude,altitude,isLab):
		self.Name = Name
		self.location = GPS()
		self.location.latitude = latitude
		self.location.longitude = longitude
		self.location.altitude = altitude
		self.status = 0
		if isinstance(isLab, str):
			self.isLab = (isLab == "True") or (isLab == "true")
		else:
			self.isLab = isLab

	def get_name(self):
		return self.Name

	def get_location(selv):
		return self.location

	def get_status(self):
		return status

	def set_status(self, status):
		self.status = status
