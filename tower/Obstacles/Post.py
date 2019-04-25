#Post Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Wall import *
from ..Angles import *
from ..Coordinate.Point import *
import math

#Finds euclidean distance of vector
#   Arguments:
#       double arg1
#       double arg2
#   Returns:
#       double dist - Euclidean distance
def euclidean(arg1, arg2):
    return math.sqrt((arg1 ** 2) + (arg2 ** 2))
#END OF euclidean()

class Posts:
    Points = [Point(42, 54), Point(-54, 54), Point(-54, -42), Point(42, -42)]
    angles = [arctan(54, 42), 3 * math.pi / 2, arctan(-54, -42), 7 * math.pi / 2]
    Distances = [math.sqrt(42**2 + 54**2), math.sqrt(2 * (54**2)), math.sqrt(42**2 + 54**2), math.sqrt(2 * (42**2))]
    INV_Distances = []
    for distance in Distances:
        INV_Distances.append( 1.0 / distance )

    #Constructs map boundaries
    #   Return:
    #       Wall[] walls - Map boundaries
    def boundaries(self):
        walls = []
        walls.append(Wall(self.Points[3], self.Points[0]))
        for x in range(1,4):
            walls.append(Wall(self.Points[x-1], self.Points[x]))
        return walls
    #END OF boundaries()

#END OF Post