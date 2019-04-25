#Tower Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Point import *
from ..Coordinate.Wall import *
from ..Coordinate.Vertex import *
from ..Angles import add as Angles_add
from ..Angles import smallestDifference, addMaybeNeg, arctan, smallestDifferencePos
from .Post import *
import math

SAFEDIST = 3

#Tower class contains tower robot data
class Tower:
    legs = [Point(6 + SAFEDIST, 6 + SAFEDIST), Point(-6 - SAFEDIST, 6 + SAFEDIST), Point(-6 - SAFEDIST, -6 - SAFEDIST), Point(6 + SAFEDIST, -6 - SAFEDIST)]
    position = Point(0,0)
    MappingStartingPos = [Point(0, 12), Point(-12, 0), Point(0, -12), Point(12, 0)]
    
    EYE_RAD = 2.0
    EYE_HEIGHT = 18.2
    THETA_U = 1.4467
    THETA_D = .71091
    THETA_Y_RANGE = THETA_U - THETA_D
    INV_THETA_Y_RANGE = 1.0 / THETA_Y_RANGE
    THETA_X_RANGE = .95655
    INV_THETA_X_RANGE = 1.0 / THETA_X_RANGE
    THETA_X_HALF = THETA_X_RANGE * .5
    THETA_L = THETA_X_HALF
    THETA_R = -THETA_X_HALF
    X_RANGE = 640.0
    Y_RANGE = 480.0
    INV_X_RANGE = 1.0 / X_RANGE
    INV_Y_RANGE = 1.0 / Y_RANGE
    
    
    POST_HEIGHT = 4.25
    POST_DIFFERENCE = 2.5
    H_DIFF = EYE_HEIGHT - POST_HEIGHT
    EYE_RAD_SQUARE = EYE_RAD**2
    TWO_EYE_RAD = EYE_RAD * 2

    #Constructor for Tower
    def __init__(self):
        self.associatedIdx = []
        self.armCalibrate = False
        self.armOrient = 0
        self.armSpeed = 0
        self.oldNormalTime = None
        self.oldOrient = None
        self.oldTime = None
        self.oldIdx = None
        self.startingPositions = []
        self.eyePos = None
    #END OF __init__()
    
    
    #Calculates and sets eye position
    def setEyePos(self):
        self.eyePos = Point(math.sin(self.armOrient) * self.EYE_RAD, math.cos(self.armOrient) * self.EYE_RAD)
    #END OF setEyePos()
    
    
    #Calculates cartesian coordinates based on pixel locations
    #   Arguments:
    #       int xValus - x location
    #       int yValue - y location
    #       double height - Object height
    #   Returns:
    #       Point location - Cartesian location
    def mapPixel(self, xValue, yValue, height):
        yRatio = float(yValue)*self.INV_Y_RANGE
        xRatio = float(xValue)*self.INV_X_RANGE
        thetaY = self.THETA_U - yRatio * self.THETA_Y_RANGE
        thetaX = self.THETA_L - xRatio * self.THETA_X_RANGE
        d = (self.EYE_HEIGHT - height) * math.tan(thetaY)
        thetaX = addMaybeNeg(self.armOrient, thetaX)
        dx = math.sin(thetaX) * d
        dy = math.cos(thetaX) * d
        return Point(self.eyePos.x + dx, self.eyePos.y + dy)
    #END OF mapPixel()
    
    
    #Calculates pixel coordinates based on cartesian coordinates
    #   Arguments:
    #       float xValue - x location
    #       float yValue - y location
    #       double height - Object height
    #   Returns:
    #       Point location - Pixel location
    def unmapPixel(self, xValue, yValue, height):
        dx = xValue - self.eyePos.x
        dy = yValue - self.eyePos.y
        d = math.sqrt(dx**2 + dy**2)
        thetaY = arctan((self.EYE_HEIGHT - height), d)
        yPos = (self.THETA_U - thetaY) * self.INV_THETA_Y_RANGE * self.Y_RANGE
        thetaX = arctan(dy, dx) - self.armOrient
        xPos = (self.THETA_L - thetaX) * self.INV_THETA_X_RANGE * self.X_RANGE
        return Point(xPos, yPos)
    #END OF unmapPixel()
    
    
    
    #Differentiates identified posts
    #   Arguments:
    #       float x - x value
    #       float y - y value
    #       posts - Post container
    #   Returns:
    #       int index - Post index
    #       float c - Eye distance
    #       float inv_thetaX - 180 - thetaX
    #       int dir - direction
    def postDifferentiate(self, x, y, posts):
        yRatio = float(y)*self.INV_Y_RANGE
        xRatio = float(x)*self.INV_X_RANGE
        thetaY = self.THETA_U - yRatio * self.THETA_Y_RANGE
        thetaX = self.THETA_L - xRatio * self.THETA_X_RANGE
        if thetaX < 0:
            dir = 1
        elif thetaX > 0:
            dir = -1
        thetaX = abs(thetaX)
        inv_thetaX = 180 - thetaX
        c = self.H_DIFF * math.tan(thetaY)
        d = math.sqrt(c**2 + self.EYE_RAD_SQUARE - self.TWO_EYE_RAD * c * math.cos(inv_thetaX))
        index = 0
        for x in range(1, 4):
            dist = posts.Distances[x]
            diff = abs(dist - d)
            if diff < self.POST_DIFFERENCE:
                index = x
                break
        return index, c, inv_thetaX, dir, d
    #END OF postDifferentiate()
    
    
    
    #Calibrates arm from post
    #   Arguments:
    #       float time - Current time
    #       int index - post index
    #       float c - Eye distance
    #       float inv_thetaX - 180 - thetaX
    #       Post posts - Post container
    #       int dir - direction
    def postCalibrate(self, time, index, c, inv_thetaX, dir, posts):
        if self.oldOrient is None:
            if index != 2:
                self.oldTime = time
                if inv_thetaX == 180:
                    self.oldOrient = posts.angles[index]
                else:
                    inv_d = posts.INV_Distances[index]
                    theta = math.asin(inv_d * c * math.sin(inv_thetaX))
                    postAng = posts.angles[index]
                    self.oldOrient = postAng + dir * theta
                    self.oldIdx = index
        else:
            if index != self.oldIdx and index != 2:
                self.oldIdx = index
                self.armCalibrate = True
                if inv_thetaX == 180:
                    self.armOrient = posts.angles[index]
                    self.armSpeed = -1 * smallestDifferencePos(self.armOrient, self.oldOrient) / (time - self.oldTime)
                    self.oldOrient = self.armOrient
                else:
                    inv_d = posts.INV_Distances[index]
                    theta = math.asin(inv_d * c * math.sin(inv_thetaX))
                    postAng = posts.angles[index]
                    self.armOrient = postAng + dir * theta
                    self.armSpeed = -1 * smallestDifferencePos(self.armOrient, self.oldOrient) / (time - self.oldTime)
                    self.oldOrient = self.armOrient
                self.oldTime = self.oldNormalTime = time
    #END OF postCalibrate()
    
    
    #Recalibrates camera arm from post
    #   Arguments:
    #       float inv_thetaX - 180 - thetaX
    #       int dir - Direction
    #       Post posts - Post container
    def postRecalibrate(self, time, c, inv_thetaX, dir, posts):
        index = -1
        diff = math.pi
        for x in range(0, 4):
            difference = smallestDifferencePos(self.armOrient, posts.angles[x])
            if difference < diff:
                diff = difference
                index = x
        inv_d = posts.INV_Distances[index]
        theta = math.asin(inv_d * c * math.sin(inv_thetaX))
        postAng = posts.angles[index]
        self.armOrient = self.oldOrient = postAng + dir * theta
        self.oldTime = self.oldNormalTime = time
    #END OF postRecalibrate()
    
    
    
    #Calculates next arm position
    #   Arguments:
    #       float time - Current time
    def recalculateArm(self, time):
        self.armOrient = addMaybeNeg(self.armOrient, self.armSpeed * (time - self.oldNormalTime))
        self.oldNormalTime = time
    #END OF recalculateArm()
    

    #Calibrates arm orient and speed
    #   Arguments:
    #       float time - Time of new orient
    #       double orient - New orient
    def calibrate(self, time, orient):
        dir, dOrient = smallestDifference(orient, self.oldOrient)
        dTime = self.oldTime - time
        self.armOrient = orient
        self.armSpeed = dOrient / dTime
        self.armCalibrate = True
    #END OF calibrate()

    
    #Decalibrates arm orient
    def decalibrate(self):
        self.armOrient = self.armSpeed = None
        self.armCalibrate = False
    #END OF decalibrate()

    
    #Emits Tower collision
    #   Arguments:
    #       int size - Current size of vertexMap list
    #   Return:
    #       Vertex[] verts - List of vertices
    #       Wall[] walls - List of walls
    def expand(self, size):
        self.associatedIdx = list(range(size, size + 4))
        verts = walls = []
        for x in range(0, 4):
            neighborIdx = []
            point = self.legs[x]
            if x % 2 == 0:
                neighborIdx.append(size + 3)
                neighborIdx.append(size + 1)
            else:
                neighborIdx.append(size)
                neighborIdx.append(size + 2)

            verts.append(Vertex(point, size + x, self.associatedIdx, neighborIdx))
        # END OF for
        walls.append(Wall(verts[3].p, verts[0].p))
        for x in range(1, 4):
            walls.append(Wall(verts[x - 1].p, verts[x].p))
        return verts, walls
    #END OF expand
#END OF Tower