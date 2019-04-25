#Block class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Point import *
from ..Coordinate.Vertex import *
from ..Coordinate.Wall import *
from ..Angles import add as Angles_add
import math


#Block class contains data on a block object
class Block:
    SAFEDIST = 7.25

    #Constructor for Block
    #   Arguments:
    #       Point loc - Location of block
    #       double orient - Orientation in radians
    #       char lett - Letter on face of block
    def __init__(self, loc, orient, lett):
        self.location = loc
        self.orientation = orient
        self.letter = lett
        self.associatedIdx = []
        self.wallIdx = []
        self.claimed = False
    #END OF __init__()

    #Generate collision and vertices based around this object
    #   Arguments:
    #       int size - Current size of vertexMap list
    #   Return:
    #       Vertex[] verts - List of vertices
    #       Wall[] walls - List of walls
    def expand(self, size, wallSize):
        self.associatedIdx = list(range(size, size + 4))
        slice = math.pi * .5
        verts = walls = []
        for x in range(0, 4):
            angle = Angles_add(self.orientation, slice * x)
            dx = math.sin(angle) * self.SAFEDIST
            dy = math.cos(angle) * self.SAFEDIST
            point = Point(self.location.x + dx, self.location.y + dy)
            verts.append(Vertex(point, size + x, associatedIdx, []))
        #END OF for
        self.wallIdx = range(wallSize, wallSize + 4)
        walls.append(Wall(verts[3].p, verts[0].p))
        for x in range(1, 4):
            walls.append(Wall(verts[x - 1].p, verts[x].p))
        return verts, walls
    #END OF expand()