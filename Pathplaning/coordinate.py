from utm import utmconv

class coordinate(object):

	def __init__(self, lat, lon):
		self.lat = lat
		self.lon = lon
		self.hight = 30

		self.converter = utmconv()

		self.hemisphere, self.zone, self.letter, self.easting, self.northing = self.converter.geodetic_to_utm(self.lat, self.lon)