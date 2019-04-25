#Mothership class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Point import *
from ..Coordinate.Vertex import *
from ..Coordinate.Wall import *
from ..Angles import arctan
from ..Angles import add as Angles_add
import math

#Mothership class contains data on the mothership object
class Mothership:
    points = [Point(12.75,0.0), Point(0.0,0.0), Point(-12.75,0.0), Point(-12.75, -7.0),
        Point(-7.25, -7.0), Point(-7.25, -9.5), Point(7.25, -9.5), Point(7.25, -7.0), Point(12.75, -7.0),
        Point(12.75, 7.0), Point(7.25, 7.0), Point(7.25, 9.5), Point(-7.25, 9.5), Point(-7.25, 7.0), Point(-12.75, 7.0)]
    
    angles = magnitudes = []
    for point in points:
        if(point.y == 0 and point.x == 0):
            angles.append(0)
        else:
            angles.append(arctan(point.y, point.x))
        magnitudes.append(math.sqrt((point.y ** 2) + (point.x ** 2)))
        
    slotPoints = [Point(0.0, 5.25), Point(-2.75, 5.25), Point(-2.75, -5.25), Point(0.0, -5.25), Point(2.75, -5.25), Point(2.75, 5.25)]
    
    
    #Constructor for Mothership
    #   Arguments:
    #       Point loc - Location of Mothership
    #       double orient - Angle of orientation in radians
    def __init__(self, loc, orient):
        self.location = loc
        self.orientation = orient
        self.associatedIdx = []
        self.destinationIdx = None
        self.destinationPoint = None
        self.exitIdx = None
        self.exitPoint = None
        self.identified = False
        self.slotAngles = []
        for point in self.slotPoints:
            self.slotAngles.append(Angles_add(arctan(point.y, point.x), orient))
        self.slotLetters = []
        self.available = True
        self.availableBy = 0
    #END OF __init__()

    #Flips the orientation of the mothership
    def flipOrient(self):
        if(self.orientation >= math.pi):
            self.orientation -= math.pi
        else:
            self.orientation += math.pi
    #END OF flipOrient()

    #Emits vertices and collision of mothership
    #   Arguments:
    #       int size - Current size of vertexMap
    #   Returns:
    #       Vertex[] verts - Vertices emitted
    #       Wall[] walls - Walls emitted
    def expand(self, size):
        self.associatedIdx = range(size, size + 15)
        self.destinationIdx = [size + 2]
        self.exitIdx = [size]
        verts = walls = []
        for x in range(0, 15):
            current = x + size
            neighborIdx = []
            if x == 1:
                point = self.location
            else:
                angle = self.angles[x]
                mag = self.magnitudes[x]
                angle = Angles_add(angle, self.orientation)
                dx = math.sin(angle) * mag
                dy = math.cos(angle) * mag
                point = Point(self.location.x + dx, self.location.y + dy)
                
            if x == 0:
                neighborIdx.append(size + 1)
                neighborIdx.append(size + 8)
                neighborIdx.append(size + 9)
                self.exitPoint = point
            elif x == 2:
                neighborIdx.append(size + 1)
                neighborIdx.append(size + 3)
                neighborIdx.append(size + 14)
                self.destinationPoint = point
            elif x == 8:
                neighborIdx.append(size)
                neighborIdx.append(size + 7)
            elif x == 9:
                neighborIdx.append(size)
                neighborIdx.append(size + 10)
            elif x == 14:
                neighborIdx.append(size + 2)
                neighborIdx.append(size + 13)
            else:
                neighborIdx.append(current + 1)
                neighborIdx.append(current - 1)
        
            verts.append(Vertex(point, current, self.associatedIdx, neighborIdx))
        #END OF for
        walls.append(Wall(verts[8].p, verts[0].p))
        walls.append(Wall(verts[16].p, verts[2].p))
        for x in range(2, 15):
            if x == 9:
                walls.append(Wall(verts[0].p, verts[x].p))
            else:
                walls.append(Wall(verts[x-1].p, verts[x].p))
        #END OF for
        return verts, walls
    #END OF expand()
#END OF Mothership