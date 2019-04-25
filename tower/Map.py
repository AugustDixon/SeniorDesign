#Map Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from .Coordinate.Point import *
from .Coordinate.Wall import *
from .Coordinate.Vertex import *
from .Coordinate.SearchTree import *
from .Obstacles.Tower import *
from .Obstacles.Post import *
from .Obstacles.Drone import *
from .Communication import *
from .Angles import add as Angles_add
from .Angles import smallestDifference, addMaybeNeg, arctan
import math
import time as tiempo

#Finds euclidean distance of vector
#   Arguments:
#       double arg1
#       double arg2
#   Returns:
#       double dist - Euclidean distance
def euclidean(arg1, arg2):
    return math.sqrt((arg1 ** 2) + (arg2 ** 2))
#END OF euclidean()

#Finds euclidean distance of two points
#   Arguments:
#       Point arg1
#       Point arg2
#   Returns:
#       double dist - Euclidean distance
def euclideanPoints(arg1, arg2):
    return math.sqrt(((arg1.x - arg2.x) ** 2) + ((arg1.y - arg2.y) ** 2))
#END OF euclideanPoints()

#Class Map contains all objects on the map and map information
class Map:
    ExpectedError = 1
    ExpectedErrorOrient = math.pi / 8
    CAMERA_CAPTURE_TIME = 5000
    DRONE_DEPOSIT_TIME = 3000
    DRONE_LIFT_TIME = 4000
    MOTHERSHIP_OFFDIST = 1
    REVERSE_START_SPEED = -50
    REVERSE_START_TIME = 2000
    PATHFIND_OVERHEAD = 1
    DRONE_PATH_COLLISION_DIST = 2.5
    HALF_PI = math.pi / 2
    THREE_HALF_PI = 3 * HALF_PI
    INV_THOUSAND = 1.0 / 1000.0
    
    DRONE_HEIGHT = 4.5
    OBSTACLE_HEIGHT = 2.785
    BLOCK_TOP_HEIGHT = 1.5
    BLOCK_MID_HEIGHT = .75
    BLOCK_CTR_CORNER = .75 * math.sqrt(2)
    MOTHERSHIP_HEIGHT = 1.5

    #Constructor for Map
    #   Arguments:
    #       int ROUND - Round number
    def __init__(self, ROUND):
        self.drones = []                                                #Container for Drone objects
        self.tower = Tower()                                            #Tower object
        self.posts = Posts()                                            #Corner posts
        self.vertexMap = []                                             #Container for map vertices
        self.collisionMap = self.posts.boundaries()                     #Container for collision, initialized with map boundaries
        legVerts, legWalls = self.tower.expand(len(self.vertexMap))     #Emits collision for Tower
        self.vertexMap.extend(legVerts)
        self.collisionMap.extend(legWalls)                              
        self.mothership = None                                          #Mothership object
        self.obstacles = []                                             #Container to hold Obstacle objects
        self.blocks = []                                                #Container to hold Block objects
        self.totalBlocks = ROUND * 2                                    #Total blocks in a round
        self.totalObstacles = ROUND * 5                                 #Total obstacles in a round
        self.droneCollisionMap = []                                     #Drone path collision map
    #END OF __init__()

    
    
    #Determines whether a drone should be added in mapping phase
    #   Arguments:
    #       Point location
    #       float orient
    #       int machineID
    def addDrone(self, location, orient, machineID):
        for drone in self.drones:
            if(drone.machineID == machineID):
                if dron.point is None:
                    drone.point = location
                    drone.orient = self.drones[x].startingOrient = orient
                    drone.startingVertexIdx = [len(self.vertexMap)]
                    self.vertexMap.append(Vertex(point, len(self.vertexMap), [], []))
    #END OF addDrone()
    
    
    
    #Checks if all drones have been identified. Helper for checkMapping()
    #   Returns:
    #       boolean
    def droneCompletion(self):
        success = True
        for drone in self.drones:
            if drone.point is None:
                success = False
        return success
    #END OF droneCompletion()
    
    
    
    #Checks if mapping phase has completed
    #   Returns:
    #       boolean success - Completion status of mapping
    def checkMapping(self):
        return ((not (self.mothership is None)) and (len(self.blocks) == self.totalBlocks) and (len(self.obstacles) >= self.totalObstacles) and self.droneCompletion())
    #END OF checkMapping()
    
    
    
    #Destroys Drone collision for specified drone
    #   Arguments:
    #       char num - Drone number
    def destroyCollision(self, num):
        for wall in self.droneCollisionMap:
            if wall.num == num:
                index = self.droneCollisionMap.remove(wall)
    #END OF destroyCollision()
    
    
    
    #Prepares for pathfinding and execution
    def executionPrep(self):
        priority = 1
        for drone in self.drones:
            if drone.machineID == 1:
                drone.priority = priority
                minDist = 1000
                lett = ' '
                for block in self.blocks:
                    
                    dist = euclideanPoints(drone.point, block.location) + euclideanPoints(self.mothership.destinationPoint, block.location)
                    if dist < minDist:
                        minDist = dist
                        lett = block.letter
                for block in self.blocks:
                    if block.letter == lett:
                        drone.block = lett
                        block.claimed = True
                        drone.destinationPoint = block.location
                        drone.destinationIdx = block.associatedIdx
                        drone.hasMothership = True
                        self.mothership.available = False
                        self.mothership.availableBy = tiempo.perf_counter() + 1000
                break
        for drone in self.drones:
            if drone.machineID != 1:
                priority += 1
                drone.priority = priority
                minDist = 1000
                lett = ' '
                for block in self.blocks:
                    if not block.claimed:
                        dist = euclideanPoints(drone.point, block.location) + euclideanPoints(self.mothership.destinationPoint, block.location)
                        if dist < minDist:
                            minDist = dist
                            lett = block.letter
                for block in self.blocks:
                    if block.letter == lett:
                        drone.block = lett
                        block.claimed = True
                        drone.destinationPoint = block.location
                        drone.destinationIdx = block.associatedIdx
    #END OF executionPrep()
    
    
    
    #Checks if drones have moved outside of expected error and fix position
    #   Arguments:
    #       Drone drone - Drone data to check
    #   Returns:
    #       boolean success - True if drone is outside of error
    def checkDronePos(self, drone):
        success = False
        for x in range(0, len(self.drones)):
            dron = self.drones[x]
            if(drone.compareID(dron)):
                if(euclidean(dron.point, drone.point) > self.ExpectedError):
                    dir, diff = smallestDifference(dron.orient, drone.orient)
                    if(diff > self.ExpectedErrorOrient):
                        self.drones[x].point = None
                        self.drones[x].orient = None
                        success = True
        return success
    #END OF checkDronePos()
    
    
    
    #Generates instructions from paths
    #   Arguments:
    #       path - Path to be taken
    #       Drone drone - Drone to be pathed
    #   Returns:
    #       Instruction[] instructions - Instructions generated
    def toInstructions(self, path, drone):
        orient = drone.orient
        point = drone.point
        drone.pathInfo = []
        currTime = 0.0
        for index in path:
            vertex = self.vertexMap[index]
            vector = Point(vertex.p.x - point.x, vertex.p.y - point.y)
            angle = arctan(vector.y, vector.x)
            magnitude = euclidean(vector.x, vector.y)
            dir, diff = smallestDifference(orient, angle)
            time = drone.makeTurn(dir, diff)
            drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
            orient = angle
            currTime += time * self.INV_THOUSAND
            speed, time = drone.makeMove(magnitude)
            drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'N'), currTime, point, orient, vertex.p, orient))
            point = vertex.p
            currTime += time * self.INV_THOUSAND
        #END OF for
        #Add closer to instructions based on state of drone and execution
        if drone.hasBlock:
            #Mount mothership and deposit block
            objPoint = self.mothership.location
            vector = Point(objPoint.x - point.x, objPoint.y - point.y)
            angle = arctan(vector.y, vector.x)
            dir, diff = smallestDifference(orient, angle)
            time = drone.makeTurn(dir, diff)
            drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
            orient = angle
            currTime += time * self.INV_THOUSAND
            speed, time = drone.mountMothership()
            drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'N'), currTime, point, orient, objPoint, orient))
            point = objPoint
            currTime += time * self.INV_THOUSAND
            if(self.mothership.identified == False):
                angle = self.mothership.slotAngles[3]
                dir, diff = smallestDifference(orient, angle)
                time = drone.makeTurn(dir, diff)
                drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
                orient = angle
                currTime += time * self.INV_THOUSAND
                speed, time = drone.makeMove(self.MOTHERSHIP_OFFDIST)
                dx = math.sin(orient) * self.MOTHERSHIP_OFFDIST
                dy = math.cos(orient) * self.MOTHERHSIP_OFFDIST
                offPoint = Point(point.x - dx, point.y - dy)
                drone.pathInfo.append(PathInfo(Instruction(b'M', -speed, time, b'N'), currTime, point, orient, offPoint, orient))
                currTime += time * self.INV_THOUSAND
                drone.pathInfo.append(PathInfo(Instruction(b'C', 0, self.CAMERA_CAPTURE_TIME, b'N'), currTime, offPoint, orient, offPoint, orient))
                currTime += self.CAMERA_CAPTURE_TIME * self.INV_THOUSAND
                drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'E'), currTime, offPoint, orient, point, orient))
                drone.endingVertexIdx = self.mothership.exitIdx[0]
                drone.finishOrient = orient
                drone.finishPoint = self.mothership.location
            else:
                index = self.mothership.slotLetters.index(drone.block)
                angle = self.mothership.slotAngles[index]
                dir, diff = smallestDifference(orient, angle)
                time = drone.makeTurn(dir, diff)
                drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
                orient = angle
                currTime += time * self.INV_THOUSAND
                speed, time = drone.makeMove(self.MOTHERSHIP_OFFDIST)
                dx = math.sin(orient) * self.MOTHERSHIP_OFFDIST
                dy = math.cos(orient) * self.MOTHERHSIP_OFFDIST
                offPoint = Point(point.x - dx, point.y - dy)
                drone.pathInfo.append(PathInfo(Instruction(b'M', -speed, time, b'N'), currTime, point, orient, offPoint, orient))
                currTime += time * self.INV_THOUSAND
                drone.pathInfo.append(PathInfo(Instruction(b'D', 0, self.DRONE_DEPOSIT_TIME, b'N'), currTime, offPoint, orient, offPoint, orient))
                currTime += self.DRONE_DEPOSIT_TIME * self.INV_THOUSAND
                drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'N'), currTime, offPoint, orient, point, orient))
                currTime += time * self.INV_THOUSAND
                exitPoint = mothership.exitPoint
                vector = Point(exitPoint.x - point.x, exitPoint.y - point.y)
                angle = arctan(vector.y, vector.x)
                dir, diff = smallestDifference(orient, angle)
                time = drone.makeTurn(dir, diff)
                drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
                currTime += time * self.INV_THOUSAND
                orient = angle
                speed, time = drone.dismountMothership()
                drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'E'), currTime, point, orient, exitPoint, orient))
                drone.endingVertexIdx = self.mothership.exitIdx[0]
                drone.finishOrient = orient
                drone.finishPoint = self.motherhsip.exitPoint
        elif drone.finished:
            #Return to starting position and undo opening actions
            angle = drone.startingOrient
            dir, diff = smallestDifference(orient, angle)
            time = drone.makeTurn(dir, diff)
            drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
            currTime += time * self.INV_THOUSAND
            orient = angle
            drone.pathInfo.append(PathInfo(Instruction(b'M', self.REVERSE_START_SPEED, self.REVERSE_START_TIME, b'N'), currTime, point, orient, point, orient))
            currTime += self.REVERSE_START_TIME * self.INV_THOUSAND
            drone.pathInfo.append(PathInfo(Instruction(b'F', 0, 0, b'E'), currTime, point, orient, point, orient))
            drone.endingVertexIdx = None
            drone.finishOrient = orient
            drone.finishPoint = point
        else:
            #Move to block and lift
            objPoint = drone.destinationPoint
            vector = Point(objPoint.x - point.x, objPoint.y - point.y)
            angle = arctan(vector.y, vector.x)
            dir, diff = smallestDifference(orient, angle)
            time = drone.makeTurn(dir, diff)
            drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
            orient = angle
            currTime += time * self.INV_THOUSAND
            drone.pathInfo.append(PathInfo(Instruction(b'L', 0, DRONE_LIFT_TIME, b'E'), currTime, point, orient, point, orient))
            drone.endingVertexIdx = path[len(path) - 1]
            drone.finishOrient = orient
            drone.finishPoint = point
    #END OF toInstructions()
    
    
    
    #Checks and emits collision for pathfinding
    #   Arguments: 
    #       path - Path to be constructed with
    #       Drone drone - Related Drone object
    #   Returns:
    #       path - Altered path
    #       Wall[] collision - Emitted walls
    def checkPathCollision(self, path, drone):
        offset = 0
        for path in drone.pathInfo:
            path.startTime += offset
            path.endTime += offset
            if(path.instruction.O == b'M'):
                moveStartTime = path.startTime + drone.startTime
                moveEndTime = path.endTime + drone.startTime
                angleLeft = Angles_add(path.startOrient, self.HALF_PI)
                dx = math.sin(angleLeft) * self.DRONE_PATH_COLLISION_DIST
                dy = math.cos(angleLeft) * self.DRONE_PATH_COLLISION_DIST
                aaLeft = Point(path.startPoint.x + dx, path.startPoint.y + dy)
                aaRight = Point(path.startPoint.x - dx, path.startPoint.y - dy)
                bbLeft = Point(path.endPoint.x + dx, path.endPoint.y + dy)
                bbRight = Point(path.endPoint.x - dx, path.endPoint.y - dy)
                BxAxLeft = bbLeft.x - aaLeft.x
                ByAyLeft = bbLeft.y - aaLeft.y
                BxAxRight = bbRight.x - aaRight.x
                ByAyRight = bbRight.y - aaRight.y
                for wall in self.droneCollisionMap:
                    if (moveStartTime <= wall.endTime) and (moveEndTime >= wall.startTime) and (drone.num != wall.droneNum):
                        if(wall.droneCollide(aaLeft, bbLeft, aaRight, bbRight, BxAxLeft, ByAyLeft, BxAxLeft, BxAxRight)):
                            index = drone.pathInfo.index(path)
                            difference = moveStartTime - wall.endTime
                            offset += difference
                            differenceInt = int(difference * 1000)
                            drone.pathInfo.insert(index, PathInfo(Instruction(b'M', 0, differenceInt, b'N'), path.startTime, path.startPoint, path.startOrient, path.startPoint, path.startOrient))
                            path.startTime += difference
                            path.endTime += difference
                self.droneCollisionMap.append(DroneWall(aaLeft, aaRight, bbLeft, bbRight, path.startTime, path.endTime, drone.num))
    #END OF emitPathCollision()
    
    
    
    #Handles all pathfinding for drones
    #   Arguments:
    #       SerialCom serialCom - Serial Communication handler
    def pathfind(self, serialCom):
        self.droneCollisionMap = []
        numDrones = len(self.drones)
        for x in range(0, numDrones):
            idx = x
            for j in range(0, numDrones):
                if self.drones[j].priority == x:
                    idx = j
                    break
            drone = self.drones[idx]
            drone.hasPath = True
            head = self.vertexMap[drone.startingVertexIdx[0]]
            pathfinder = SearchTree(head, drone.destinationIdx, drone.destinationPoint, self.vertexMap)
            drone.startTime = tiempo.perf_counter() + self.PATHFIND_OVERHEAD
            path = pathfinder.pathfind()
            self.toInstructions(path, drone)
            self.checkPathCollision(path, drone, droneCollisionMap)
            drone.finishTime = drone.startTime + drone.pathInfo[len(drone.pathInfo) - 1].endTime
            while(tiempo.perf_counter() < drone.startTime):
                pass
            serialCom.transmitInstructions(drone.num, drone.emitInstructions())
    #END OF pathfind()
    
    
    
    #Handles transitions between execution stages for each drone
    #   Arguments:
    #       SerialCom serialCom - Serial Communication handler
    def checkDroneStatus(self, serialCom):
        time = tiempo.perf_counter()
        if time > self.mothership.availableBy:
            self.mothership.available = True
        
        for drone in self.drones:
            if (not drone.recalculate) and (time > drone.finishTime):
                if drone.hasPath:
                    self.destroyCollision(drone.num)
                    drone.hasPath = False
                pathfind = False
                lockMothership = False
                if drone.finished:
                    pass
                elif drone.hasBlock:
                    #Just deposited block at mothership
                    pathfind = True
                    drone.hasBlock = False
                    drone.hasMothership = False
                    available = False
                    for block in self.blocks:
                        if not block.claimed:
                            block.claimed = available = True
                            drone.block = block.letter
                            drone.destinationIdx = block.associatedIdx
                            drone.destinationPoint = block.location
                    if not available:
                        drone.block = ' '
                        drone.finished = True
                        drone.destinationIdx = drone.startingVertexIdx
                        drone.destinationPoint = self.vertexMap[drone.startingVertexIdx[0]].p
                elif mothership.available or drone.hasMothership:
                    #Just picked up block
                    drone.hasBlock = True
                    lockMothership = True
                    self.mothership.available = False
                    drone.hasMothership = True
                    drone.destinationPoint = self.mothership.destinationPoint
                    drone.destinationIdx = self.mothership.destinationIdx
                    
                if pathfind:
                    drone.hasPath = True
                    head = self.vertexMap[endingVertexIdx]
                    pathfinder = SearchTree(head, drone.destinationIdx, drone.destinationPoint, self.vertexMap)
                    drone.startTime = tiempo.perf_counter() + self.PATHFIND_OVERHEAD
                    path = pathfinder.pathfind()
                    self.toInstructions(path, drone)
                    self.checkPathCollision(path, drone, droneCollisionMap)
                    drone.finishTime = drone.startTime + drone.pathInfo[-1].endTime
                    if drone.hasMothership:
                        if self.mothership.identified:
                            index = -8
                        else:
                            index = -1
                        finishTime = drone.pathInfo[index].endTime
                        self.mothership.availableBy = drone.startTime + finishTime
                    while(tiempo.perf_counter() < drone.startTime):
                        pass
                    serialCom.transmitInstructions(drone.num, drone.emitInstructions())
                    
    #END OF checkDroneStatus()
    
    
    
    #Receives identified mothership letter and finishes first deposition
    #   Arguments:
    #       char letter - Identified letter
    #       SerialCom serialCom - Serial Communication handler
    def mothershipIdentLetter(self, letter, serialCom):
        drone = None
        for dr in self.drones:
            if dr.machineID == 1:
                drone = dr
                break
        if letter == 'B':
            self.mothership.slotLetter = ['B', 'C', 'D', 'E', 'F', 'A']
        else:
            self.mothership.slotLetter = ['E', 'F', 'A', 'B', 'C', 'D']
        self.motherhsip.identified = True
        
        #Finish deposition
        index = self.mothership.slotLetter.index(drone.block)
        angle = self.motherhsip.slotAngles[index]
        drone.pathInfo = []
        dir, diff = smallestDifference(drone.finishOrient, angle)
        time = drone.makeTurn(dir, diff)
        point = drone.finishPoint
        drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), 0.0, point, drone.finishOrient, point, angle))
        orient = angle
        currTime = time * self.INV_THOUSAND
        speed, time = drone.makeMove(self.MOTHERSHIP_OFFDIST)
        dx = math.sin(orient) * self.MOTHERSHIP_OFFDIST
        dy = math.cos(orient) * self.MOTHERHSIP_OFFDIST
        offPoint = Point(point.x - dx, point.y - dy)
        drone.pathInfo.append(PathInfo(Instruction(b'M', -speed, time, b'N'), currTime, point, orient, offPoint, orient))
        currTime += time * self.INV_THOUSAND
        drone.pathInfo.append(PathInfo(Instruction(b'D', 0, self.DRONE_DEPOSIT_TIME, b'N'), currTime, offPoint, orient, offPoint, orient))
        currTime += self.DRONE_DEPOSIT_TIME * self.INV_THOUSAND
        drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'N'), currTime, offPoint, orient, point, orient))
        currTime += time * self.INV_THOUSAND
        exitPoint = mothership.exitPoint
        vector = Point(exitPoint.x - point.x, exitPoint.y - point.y)
        angle = arctan(vector.y, vector.x)
        dir, diff = smallestDifference(orient, angle)
        time = drone.makeTurn(dir, diff)
        drone.pathInfo.append(PathInfo(Instruction(b'T', dir, time, b'N'), currTime, point, orient, point, angle))
        currTime += time * self.INV_THOUSAND
        orient = angle
        speed, time = drone.dismountMothership()
        drone.pathInfo.append(PathInfo(Instruction(b'M', speed, time, b'E'), currTime, point, orient, exitPoint, orient))
        currTime += time * self.INV_THOUSAND
        drone.finishOrient = angle
        drone.finishPoint = self.mothership.exitPoint
        drone.startTime = perf_counter()
        drone.finishTime = drone.startTime + currTime
        serialCom.transmitInstructions(drone.num, drone.emitInstructions())
    #END OF motherhsipIdentLetter()
    
    
    
    #Calculates expected drone position
    #   Arguments:
    #       double frameCollectTime - Time last image was grabbed
    def calculateDronePos(self, frameCollectTime):
        for drone in self.drones:
            for pathInfo in drone.pathInfo:
                if (frameCollectTime >= pathInfo.startTime) and (frameCollectTime < pathInfo.endTime):
                    if pathInfo.instruction.O == b'M':
                        timeDifference = pathInfo.endTime - pathInfo.startTime
                        ratio = (frameCollectTime - pathInfo.startTime) / timeDifference
                        mag = ratio * pathInfo.dist
                        dx = math.sin(pathInfo.startOrient) * mag
                        dy = math.cos(pathInfo.startOrient) * mag
                        drone.point = Point(pathInfo.startPoint.x + dx, pathInfo.startPoint.y + dy)
                    elif pathInfo.instruction.O == b'T':
                        timeDifference = pathInfo.endTime - pathInfo.startTime
                        ratio = (frameCollectTime - pathInfo.startTime) / timeDifference
                        angle = ratio * pathInfo.angle
                        drone.orient = addMaybeNeg(pathInfo.startOrient, angle)
    #END OF calculateDronePos()
    
    
    
    #Handles pathfinding recalculations
    #   Arguments:
    #       SerialCom serialCom - Serial Communication handler
    def recalculate(self, serialCom):
        for drone in self.drones:
            if drone.recalculate == True and not (drone.point is None):
                drone.recalculate = False
                head = Vertex(drone.point, len(self.vertexMap), [], [])
                head.droneExpansion()
                pathfinder = SearchTree(head, drone.destinationIdx, drone.destinationPoint, self.vertexMap)
                drone.startTime = tiempo.perf_counter() + self.PATHFIND_OVERHEAD
                path = pathfinder.pathfind()
                self.toInstructions(path, drone)
                self.checkPathCollision(path, drone, droneCollisionMap)
                drone.finishTime = drone.startTime + drone.pathInfo[-1].endTime
                if drone.hasMothership:
                    if self.mothership.identified:
                        index = -8
                    else:
                        index = -1
                    finishTime = drone.pathInfo[index].endTime
                    self.mothership.availableBy = drone.startTime + finishTime
                while(tiempo.perf_counter() < drone.startTime):
                    pass
                serialCom.transmitInstructions(drone.num, drone.emitInstructions())
    #END OF recalculate()

#END OF Map