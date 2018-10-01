import numpy as np

class coordinate(object):

	#self.lat
	#self.lon
	#self.coord = np.array([self.lati, self.longi])

	def __init__(self, lat, lon):
		self.lat = lat
		self.lon = lon
		self.coord = np.array([lat, lon])