from utm import utmconv

class coordinate(object):

	def __init__(self, lat=0, lon=0, northing=0, easting=0):
		self.lat = lat
		self.lon = lon
		self.northing = northing
		self.easting = easting
		self.altitude = 30

		# Default values:
		self.hemisphere = 'N'
		self.zone = 32
		self.letter = 'U'

		self.converter = utmconv()

		if self.lat == 0 and self.lon == 0:
			self.update_geo_coordinates()
		if self.northing == 0 and self.easting == 0:
			self.update_UTM_coordinates()

	def update_UTM_coordinates(self):
		self.hemisphere, self.zone, self.letter, self.easting, self.northing = self.converter.geodetic_to_utm(self.lat,
																											  self.lon)

	def update_geo_coordinates(self):
		self.lat, self.lon = self.converter.utm_to_geodetic(self.hemisphere, self.zone, self.easting, self.northing)
