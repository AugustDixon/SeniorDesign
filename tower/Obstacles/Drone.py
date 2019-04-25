#Drone Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from ..Coordinate.Point import *
from ..Coordinate.Vertex import * 
import math

INV_THOUSAND = 1 / 1000

#Finds euclidean distance of two points
#   Arguments:
#       Point arg1
#       Point arg2
#   Returns:
#       double dist - Euclidean distance
def euclidean(arg1, arg2):
    return math.sqrt(((arg1.x - arg2.x) ** 2) + ((arg1.y - arg2.y) ** 2))
#END OF euclidean()

class Instruction:
    def __init__(self, opcode, arg1, arg2, special):
        self.O = opcode     #Already in Byte form
        self.Arg1 = arg1
        self.Arg2 = arg2
        self.S = special    #Already in Byte form
    #END OF __init__()
#END OF Instruction

#PathInfo contains instructions and metadata on them
class PathInfo:

    def __init__(self, instruct, prevTime, initPoint, initOrient, nextPoint, nextOrient):
        self.intruction = instruct
        self.startTime = prevTime
        self.endTime = prevTime + instruct.Arg2 * INV_THOUSAND
        self.startPoint = initPoint
        self.endPoint = nextPoint
        self.startOrient = initOrient
        self.endOrient = nextOrient
        if initPoint.equals(nextPoint):
            self.dist = 0
        else:
            self.dist = euclidean(initPoint, nextPoint)
        if initOrient == nextOrient:
            self.angle = 0
        else:
            dir, self.angle = Angles.smallestDifference(initOrient, nextOrient)
            self.angle = self.angle * dir
    #END OF __init__()
#END OF PathInfo

#DroneWall is a version of Wall for drone collision
class DroneWall:
    def __init__(self, aaLeft, aaRight, bbLeft, bbRight, prevTime, nextTime, num):
        self.aLeft = aaLeft
        self.bLeft = bbLeft
        self.aRight = aaRight
        self.bRight = bbRight
        self.startTime = prevTime
        self.endTime = nextTime
        self.droneNum = num
    #END OF __init__()
    
    #Checks for collisions
    #   Arguments:
    #       Point aaLeft, bbLeft - Points of left wall
    #       Point aaRight, bbRight - Points of Right wall
    #       double BxAxLeft, ByAyLeft - Precomputed values of left wall
    #       double BxAxRight, ByAyRight - Precomputed values of right wall
    #   Returns:
    #       boolean result
    def droneCollide(self, aaLeft, bbLeft, aaRight, bbRight, BxAxLeft, ByAyLeft, BxAxRight, ByAyRight):
        if Vertex.intersect(aaLeft, bbLeft, self.aLeft, self.bLeft, BxAxLeft, ByAyLeft):
            return True
        elif Vertex.intersect(aaLeft, bbLeft, self.aRight, self.bRight, BxAxLeft, ByAyLeft):
            return True
        elif Vertex.intersect(aaRight, bbRight, self.aLeft, self.bLeft, BxAxRight, ByAyRight):
            return True
        else:
            return Vertex.intersect(aaRight, bbRight, self.aRight, self.bRight, BxAxRight, ByAyRight)
    #END OF droneCollide()
#END OF DroneWall


#Drone class to contain Drone data
class Drone:
    
    
    #Constructor for Drone
    #   Arguments:
    #       char number - Comms identifier
    #       int machine - Machine ID number
    def __init__(self, number, machine):
        self.num = number
        self.machineID = machine
        self.priority = None
        
        self.point = None
        self.orient = None
        
        self.recalculate = False
        
        self.mapped = False
        self.hasBlock = False
        self.finished = False
        self.hasMothership = False
        
        self.block = ' '
        
        self.startingVertexIdx = None
        self.startingOrient = None
        
        self.endingVertexIdx = None
        self.destinationPoint = None
        self.destinationIdx = []
        
        self.pathInfo = []
        self.hasPath = False
        
        self.startTime = None
        self.finishTime = None
        self.finishOrient = None
        self.finishPoint = None
    #END OF __init__()
    
    #Compares two drones' machine ID 
    #   Arguments:
    #       Drone drone - Drone to be compared
    #   Returns:
    #       boolean
    def compareID(self, drone):
        return (drone.machineID == self.machineID)
    #END OF compareID()
    
    #Calculates time from angle for turn commands
    #   Arguments:
    #       double angle - Angle to turn
    #       int dir - Direction to turn
    #   Returns:
    #       long time - Time in ms
    def makeTurn(self, angle, dir):
        pass#TODO
    #END OF makeTurn()
    
    #Calculates the time and speed for move commands
    #   Arguments:
    #       double dist - Distance to be traveled
    #   Returns:
    #       int speed
    #       long time - Time in ms
    def makeMove(self, dist):
        pass#TODO
    #END OF makeMove()   

    #Generates speed and time for mothership mounting
    #   Returns:
    #       int speed
    #       long time - Time in ms
    def mountMothership(self):
        pass#TODO
    #END OF mountMothership()
    
    #Generates speed and time for mothership dismounting
    #   Returns:
    #       int speed
    #       long time - Time in ms
    def dismountMothership(self):
        pass#TODO
    #END OF dismountMothership()
    
    #Emits instruction list
    #   Returns:
    #       Instruction[] instructions
    def emitInstructions(self):
        instructions = []
        for path in self.pathInfo:
            instructions.append(path.instruction)
        return instructions
    #END OF emitInstructions()
    
    def createInstruction(self, code, int, long, end):
        return Instruction(code, int, long, end)
    #END OF createInstruction()
#END OF Drone