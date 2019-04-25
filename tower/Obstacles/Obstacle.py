#Obstacle Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Point import *
from ..Coordinate.Wall import *
from ..Coordinate.Vertex import *
import math

class Obstacle:
    #Constants
    RADIUS = 4          #Vertex distance from center (inches)
    VERTICES = 5        #Number of Vertices produced, odd numbers produce better results

    #Constructor
    #   Arguments:
    #       Point loc - Point of obstacle location
    def __init__(self, loc):
        self.location = loc
        self.associatedIdx = []
    #END OF __init__()

    #Generate collision and vertices based around this object
    #   Arguments:
    #       int size - Current size of vertexMap list
    #   Return:
    #       Vertex[] verts - List of vertices
    #       Wall[] walls - List of walls
    def expand(self, size):
        SPAN = self.VERTICES - 1
        end = size + SPAN
        slice = 2 * math.pi / self.VERTICES
        self.associatedIdx = list(range(size, size + self.VERTICES))
        verts = walls = []
        for x in range(0, self.VERTICES):
            current = size + x
            neighborIdx = []
            angle = x * slice
            dx = math.sin(angle) * self.RADIUS
            dy = math.cos(angle) * self.RADIUS
            point = Point(self.location.x + dx, self.location.y + dy)
            if x == 0:
                neighborIdx.append(end)
                neighborIdx.append(size + 1)
            elif x == end:
                neighbotIdx.append(size)
                neighborIdx.append(end - 1)
            else:
                neighborIdx.append(current - 1)
                neighborIdx.append(current + 1)
            verts.append(Vertex(point, current, self.associatedIdx, neighborIdx))
        #END OF for
        walls.append(Wall(verts[SPAN].p, verts[0].p))
        for x in range(1, self.VERTICES):
            walls.append(Wall(verts[x-1].p, verts[x].p))
        return verts, walls
    #END OF expand()

#END OF Obstacle