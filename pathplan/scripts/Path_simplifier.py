from coordinate import Coordinate
from math import sqrt


class PathSimplifier(object):
    def __init__(self, path=[], step_size=128, num_of_points=10):
        self.path = path
        self.path_simplified = []
        self.step_size = step_size
        self.num_of_points = num_of_points

    def delete_with_step_size(self):
        ''' Not a safe function to use IRL, but nice starting point for testing.
        It just deletes points in self.path, with the spacing of self.step_size '''
        path_temp = []
        count = 0
        for i in self.path:
            if count == 0:
                path_temp.append(i)
            count += 1
            if count == self.step_size:
                count = 0
        self.path_simplified = path_temp
        self.path_simplified.append(self.path[-1])

    def delete_with_step_size_safe(self, threshold=2):
        ''' This function will take two points with distance of 'self.step_size' and delete all points in between,
         if all points are within the perpendicular distance of 'threshold' to the line between the two points.
         If not, the distance will be halfed, and tried again. '''
        i = 0
        path_temp = []
        step_size_temp = self.step_size
        if (i+self.step_size) < len(self.path):
            while (i + self.step_size) < len(self.path):
                i1 = int(i)
                p1 = self.path[i1]
                i2 = int(i1 + step_size_temp)
                p2 = self.path[i2]
                outlier = False
                # Take all points in between p1 and p2 and calculate the perpendicular distance to the line between p1 and
                # p2. p3 is the point in question, p4 is a point on the line, that in conjunction with p3 makes a line
                # perpendicular to the line between p1 and p2
                for j in range(i1, i2):
                    p3 = self.path[j]
                    # Taken from: https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
                    k = ((p2.northing - p1.northing) * (p3.easting - p1.easting) -
                         (p2.easting - p1.easting) * (p3.northing - p1.northing)) / \
                        ((p2.northing - p1.northing) ** 2 + (p2.easting - p1.easting) ** 2)
                    east = p3.easting - k * (p2.northing - p1.northing)
                    nort = p3.northing - k * (p2.easting - p1.easting)
                    p4 = Coordinate(easting=east, northing=nort)
                    if self.distance(p3, p4) > threshold:
                        outlier = True
                        break
                if outlier:
                    # Half the step size and try again.
                    step_size_temp /= 2
                else:
                    # All good, move on.
                    path_temp.append(p1)
                    i += step_size_temp
                    step_size_temp = self.step_size
        else:
            path_temp.append(self.path[0])
        # End by adding the end point
        path_temp.append(self.path[-1])
        self.path_simplified = path_temp

    def distance(self, p1, p2):
        return sqrt((p2.easting - p1.easting)**2 + (p2.northing - p1.northing)**2)

    def get_simple_path(self):
        return self.path_simplified

