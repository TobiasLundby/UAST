from AQLogReader import aqLogReader
from math import sqrt, pi, sin, cos
from export_to_qgc import export_to_qgc
from export_kml import export_kml

class PlanCleaner:

    # loads the log from the file
    def __init__(self, fileName):
        fileHandler = aqLogReader(fileName)
        self.data = fileHandler.getData()
        print(self.data.__len__())

    # calculates distance from line to point in meters
    def dist(self, start, end, point):
        px = end[0] - start[0]
        py = end[1] - start[1]
        pz = end[2] - start[2]

        ux = ((point[0] - start[0]) * px)
        uy = ((point[1] - start[1]) * py)
        uz = ((point[2] - start[2]) * pz)

        if (px * px + py * py + pz * pz) == 0:
            u = 1
        else:
            u = (ux + uy + uz) / (px * px + py * py + pz * pz)

            if u > 1:
                u = 1
            elif u < 0:
                u = 0

        dx = start[0] + u * px - point[0]
        dy = start[1] + u * py - point[1]
        dz = start[2] + u * pz - point[2]

        return sqrt(dx * dx + dy * dy + dz * dz)

    # converts geodetic coordinates to utm and calls dist
    def distGeodetic(self, start, end, point):
        return self.dist(start, end, point)

    # tests if all points in the array can fit one line by threshold T
    def fit(self, array, maxError):
        for i in range(1, array.__len__() - 1):
            if self.distGeodetic(array[0], array[array.__len__() - 1], array[i]) > maxError:
                return False
        return True

    # remove unnecessary waypoints
    def cleanPath(self, maxError):
        startIndex = 0
        endIndex = 1

        while (True):

            # End of array: remove all elements from last startIndex to the end of the list
            if (endIndex + 1) >= (self.data.__len__()):
                del self.data[startIndex + 1: endIndex]
                break

            # Increment the end pointer if a line fit for the list between start index and end index
            if self.fit(self.data[startIndex:endIndex + 1], maxError):
                endIndex = endIndex + 1

            # All data from start index to the previous end index can fit a line
            # Therfore delete all elements wetween start and end and setup the indexes for new iteration
            else:
                del self.data[startIndex + 1: endIndex - 1]
                startIndex = endIndex - 1
                endIndex = startIndex + 1

    # return route plan
    def getPlan(self):
        return self.data

    # write log to file in 2d
    def write_list2D(self, filename):
        writer = export_to_qgc(filename)
        writer.write_list2D(self.data)

    # write log to file in 3d
    def write_list3D(self, filename):
        writer = export_to_qgc(filename)
        writer.write_list3D(self.data)
