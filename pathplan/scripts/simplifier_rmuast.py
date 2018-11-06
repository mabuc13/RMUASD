import csv
import numpy as np
from numpy.linalg import norm
from numpy import argmin, dot, arccos
# from matplotlib import pyplot as plt
from math import sqrt, pow
from coordinate import Coordinate
# from utm import utmconv
from collections import deque


def triangularAreaFromPoints(p1, p2, p3):
    return norm(np.cross(p2 - p1, p3 - p1)) / 2
    # return abs(np.cross(p2 - p1, p3 - p1)) / 2


def triangularAreasFromArray(array):
    # calculates the area spanned by the triangle formed by three consecutive points in the list
    # the first and the last point are set to infinity
    # lists of 3 consecutive points in the array
    p1 = array[:-2]
    p2 = array[1:-1]
    p3 = array[2:]

    # calculate areas with cross product
    areas = norm(np.cross(p2 - p1, p3 - p1)) / 2
    # areas = abs(np.cross(p2 - p1, p3 - p1)) / 2

    result = np.empty((len(array),), array.dtype)
    result[0] = np.inf
    result[-1] = np.inf
    result[1:-1] = areas

    return result


def perpDistance(line, point):
    # the line is represented by two points in a tuple
    # the formula is just a standard vector formula for distance from point to line
    distance = norm(np.cross(line[1] - line[0], point - line[0])) / norm(line[1] - line[0])

    return distance


def vectorAngleDiff(v1, v2):
    angle = arccos(dot(v1, v2)/(norm(v1)*norm(v2)))
    deg = np.degrees(angle)

    return abs(deg)


class FlightPlanner(object):

    # Max speed in m/s
    maxSpeed = 30

    def __init__(self):
        self.data = np.empty((0, 9), float)
        self.utm_data = None
        self.simplified = None

    # def importData(self, file_name):
    #     try:
    #         with open(file_name, newline='') as csvfile:
    #             csv_in = csv.reader(csvfile, delimiter=',', quotechar='|')
    #             for i, row in enumerate(csv_in):
    #                 if i < 1:
    #                     continue

    #                 if row[0] == '#time_boot':
    #                     break

    #                 row = [float(item) for item in row]

    #                 self.data = np.append(self.data, np.array([row]), axis=0)
    #     except Exception as exc:
    #         # print(exc)
    #         raise exc

    #     self.utm_data = self.convertToUTM()

    def loadPath(self, path, altitude=30):
        utm = [[coord.easting,coord.northing] for coord in path]

        # create len(path) x 3 array initialized with altitude value all over
        self.utm_data = np.full((len(path),3), altitude, float)
        # insert x,y values into the first two columns
        self.utm_data[:,:-1] = np.array([utm])
        # print(self.utm_data)

        # self.utm_data = np.array([path])

    def getSimpleCoordinates(self):

        print("converting")
        print(self.simplified)
        # coordinates = [xy for xy in self.simplified[:,0:2]]
        coordinates = [Coordinate(easting=xy[0],northing=xy[1]) for xy in self.simplified[:,0:2]]
        print(coordinates)
        # print(type(coordinates))
        # print(len(coordinates))
        return coordinates

    def simplifyByDistance(self, dist):
        coords = self.utm_data#[:,1:4]
        self.simplified = self.douglasPeucker(coords,dist)

        return self.simplified

    def simplifyByWaypointCount(self, waypoints):
        coords = self.utm_data#[:,1:4]

        areas = self.VWAlgorithm(coords)
        ordered = sorted(areas,reverse=True)
        # print(len(ordered))
        # print(self.utm_data.shape)
        threshold = ordered[int(waypoints)]
        # print(ordered)
        self.simplified = coords[areas > threshold]
        # print(self.simplified.shape)

        return self.simplified

    def simplifyByDistanceAndAngle(self, dist, angle, angleFiltering):
        coords = self.utm_data#[:, 1:4]
        self.simplified = self.douglasPeucker(coords, dist, angle, angleFiltering)

        return self.simplified

    def simplifyFlightPlan(self, dist=None, waypoints=None, angle=None, angleFiltering=1):

        if waypoints != None:
            self.simplified = self.simplifyByWaypointCount(waypoints)

        elif angle != None:
            if dist == None:
                self.simplified = self.simplifyByDistanceAndAngle(np.inf,angle,angleFiltering)
            else:
                self.simplifyByDistanceAndAngle(dist,angle,angleFiltering)

        elif dist != None:
            self.simplified = self.simplifyByDistance(dist)

        else:
            # Base case is just a max distance of 2 meters away from original route
            self.simplified = self.simplifyByDistance(2)

        print("Number of waypoints before simplification: {}".format(len(self.utm_data)))
        print("Number of waypoints after simplification: {}".format(len(self.simplified)))

        return self.simplified

    def douglasPeucker(self, points, eps, angle=None, stepSize=1):
        d_max = 0
        d_index = 0
        a_max = 0
        a_index = 0
        end = len(points)
        recResults = None

        distList = deque()
        bearing_end2end = points[end - 1] - points[0]

        for i in range(1, end):
            d = perpDistance((points[0],points[end - 1]),points[i])

            # Keep track of how far you've travelled. We only want to look at the bearing deviation for a vector that
            # represents a large enough distance. Vectors that are small are vulnerable to noise
            travelled_d = norm(points[i] - points[i-1])
            distList.append(travelled_d)
            total_dist = sum(distList)


            if d > d_max:
                d_index = i
                d_max = d
            if angle != None and total_dist > stepSize:
                bearing_current = points[i] - points[i-len(distList)]
                a = vectorAngleDiff(bearing_current, bearing_end2end)
                if a > a_max:
                    a_index = int(i - len(distList)/2)
                    if a_index == 0:
                        a_index = 1
                    a_max = a

                # remove elements from distance list until it is below the threshold again (FIFO)
                while sum(distList) >= stepSize:
                    distList.popleft()


        if d_max > eps:
            # subdivide array further until distance condition is satisfied
            # add one to the index so the end of recResults1 and the start of recResults2 are the same
            recResults1 = self.douglasPeucker(points[0:d_index+1,:], eps, angle, stepSize)
            recResults2 = self.douglasPeucker(points[d_index:end,:], eps, angle, stepSize)

            # discard the first coordinate of the second result because it is equal to the last coordinate
            # in the first result
            recResults = np.append(recResults1,recResults2[1:],axis=0)

        elif angle != None and a_max > angle:
            recResults1 = self.douglasPeucker(points[0:a_index+1, :], eps, angle, stepSize)
            recResults2 = self.douglasPeucker(points[a_index:end, :], eps, angle, stepSize)

            recResults = np.append(recResults1, recResults2[1:], axis=0)

        else:
            # remove all points in between the ends
            recResults = np.array([points[0],points[end-1]])

        return recResults

    def VWAlgorithm(self,coordinates):
        nmax = len(coordinates)

        # Calculate the areas of the triangles and make another array with indeces,
        # so we can access the right points in the original array as entries in the area list are deleted
        areas = triangularAreasFromArray(coordinates)
        indeces = list(range(nmax))

        # Make a new array where all the updated areas can be stored as we go.
        # Entries in the original area list will be removed as the algorithm runs
        real_areas = np.copy(areas)

        # find the point with the smallest associated area and its index. Argmin returns the index
        min_area_idx = argmin(areas)
        min_area_val = areas[min_area_idx]

        # Remove this entry from the area list and the index list
        areas = np.delete(areas,min_area_idx)
        indeces.pop(min_area_idx)

        # Now we need to recalculate the triangle areas to the left and right of the removed entry
        # this process will continue until only the start point and the end point remains
        while min_area_val < np.inf:
            # Update triangle to the right in the list
            try:
                # min_area_idx will point to the point to the right in the list, because of the deletion
                right_idx = indeces[min_area_idx]
                right_area = triangularAreaFromPoints(coordinates[indeces[min_area_idx - 1]],
                                                      coordinates[indeces[min_area_idx]],
                                                      coordinates[indeces[min_area_idx + 1]])
            except IndexError:
                # trying to update area of endpoint. Don't do it
                pass
            else:
                # the area of the triangle might have decreased after the deletion, so it is capped at
                # the previously smallest value to preserve the proper sequencing
                if right_area <= min_area_val:
                    right_area = min_area_val

                # update area values in both arrays
                real_areas[right_idx]     = right_area
                areas[min_area_idx]     = right_area

            # Update triangle to the left in the list
            if min_area_idx > 1:
                left_idx = indeces[min_area_idx - 1]
                left_area = triangularAreaFromPoints(coordinates[indeces[min_area_idx - 2]],
                                                     coordinates[indeces[min_area_idx - 1]],
                                                     coordinates[indeces[min_area_idx]])

                # the area of the triangle might have decreased after the deletion, so it is capped at
                # the previously smallest value to preserve the proper sequencing
                if left_area <= min_area_val:
                    left_area = min_area_val

                # update area values in both arrays
                real_areas[left_idx]     = left_area
                areas[min_area_idx - 1]    = left_area

            # Find the new smallest area and remove the entry from area list and index list
            min_area_idx = argmin(areas)
            min_area_val = areas[min_area_idx]
            areas = np.delete(areas,min_area_idx)
            indeces.pop(min_area_idx)

        return real_areas

    # def exportPlan(self):
    #     uc = utmconv()
    #     geodetic = np.empty((0,2),float)

    #     with open('plan.csv', 'w', newline='') as csvfile:
    #         writer = csv.writer(csvfile, delimiter=',',
    #                                 quotechar='|', quoting=csv.QUOTE_MINIMAL)
    #         for row in self.simplified:
    #             (lat,lon) = uc.utm_to_geodetic(self.hemisphere,self.zone,row[0],row[1])

    #             writer.writerow([lat,lon,row[2]])

    # def plotUTM(self):
    #     plt.plot(self.utm_data[:, 1], self.utm_data[:, 2])
    #     plt.axis('equal')
    #     plt.xlabel('East')
    #     plt.ylabel('North')
    #     plt.grid()
