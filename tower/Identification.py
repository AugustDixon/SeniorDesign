#Identification class definition file
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72



import numpy as np
import cv2 as cv
import keras
from keras.models import Sequential
from keras.layers import Dense, Activation
import math
from .Coordinate import *
from .Coordinate.Point import *
from .Obstacles.Mothership import *
from .Obstacles.Block import *
from .Obstacles.Obstacle import *
from .Obstacles.Tower import *
from .Obstacles.Drone import *
from .Angles import add as Angles_add
from .Angles import smallestDifference, addMaybeNeg, arctan, smallestDifferencePos


#Finds euclidean distance of two points
#   Arguments:
#       Point arg1
#       Point arg2
#   Returns:
#       double dist - Euclidean distance
def euclideanPoints(arg1, arg2):
    return math.sqrt(((arg1.x - arg2.x) ** 2) + ((arg1.y - arg2.y) ** 2))
#END OF euclideanPoints()



#Class Identification handles most complex object identification and image processing algorithms as well as constants used
class Identification:
    LowerLED = np.array([0, 0, 245])
    UpperLED = np.array([180, 51, 255])
    
    LowerMothershipLED = np.array([80, 102, 178])
    UpperMothershipLED = np.array([90, 204, 255])
    
    LowerBlockObstacle = np.array([0, 0, 165])
    UpperBlockObstacle = np.array([180, 51, 255])
    
    LowerBlockFace = np.array([0, 0, 115])
    UpperBlockFace = np.array([180, 76, 255])
    
    LowerBlockSurr = np.array([10, 90, 140])
    UpperBlockSurr = np.array([20, 165, 192])
    
    #Drone Constants
    DRONE_BACK_BACK = 1.906
    DRONE_BACK_FRONT = 3.3
    DRONE_FRONT_CENTER = 1.5
    
    #Mothership Constants
    MOTHERSHIP_LED_DIST = 2.5
    MOTHERSHIP_LED_CENTR = 7
    MOTHERSHIP_LED_DBL = 2 * MOTHERSHIP_LED_DIST
    
    BLOCK_FACE_Y = 44
    HALF_BLOCK_Y = int(.5 * BLOCK_FACE_Y)
    BLOCK_FACE_X = 54
    HALF_BLOCK_X = int(.5 * BLOCK_FACE_X)
    
    DRONE_IMAGE_Y = 120
    DRONE_IMAGE_X = 160
    
    THREE_HALF_PI = 1.5 * math.pi
    HALF_PI = .5 * math.pi
    QUARTER_PI = .25 * math.pi
    TO_DEGREE = 180.0 / math.pi
    Expected_Error = 1
    Ident_Threshold = .5
    MIN_DIST = 6
    
    X_MIN = 80
    X_MAX = 640 - X_MIN
    
    trainingPath = '/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/trainingData/'
    
    #Constructor for Identification
    #   Arguments:
    #       cv.VideoCapture cam - Camera object for OpenCV
    #       File LogFile - Logging file for performance reporting
    def __init__(self, dataCollect, LogFile, EX_LOG):
        self.blocksIdentified = 0
        self.Logging = LogFile
        self.EX_LOG = EX_LOG
        self.dataCollect = dataCollect
        
        ###BLOB DETECTORS
        # Setup LEDBlobDetector parameters
        params = cv.SimpleBlobDetector_Params()
        # Change thresholds
        # Image will be thresholded beforehand, meaning only one thresholding step is necessary
        params.minThreshold = 0
        params.maxThreshold = 256

        # Filter by Area
        params.filterByArea = True
        params.minArea = 7
        params.maxArea = 40

        # Filter by Circularity
        params.filterByCircularity = False
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = .5
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = .1

        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            self.LEDBlobDetector = cv.SimpleBlobDetector(params)
        else : 
            self.LEDBlobDetector = cv.SimpleBlobDetector_create(params)
    
    
        # Filter by Area
        params.filterByArea = True
        params.minArea = 300
        params.maxArea = 1600
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = .70
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = .4
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = .05
    
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            self.ObstacleBlobDetector = cv.SimpleBlobDetector(params)
        else : 
            self.ObstacleBlobDetector = cv.SimpleBlobDetector_create(params)
        
        # Filter by Area
        params.filterByArea = True
        params.minArea = 20
        params.maxArea = 350

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = .1
        params.maxCircularity = .70
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = .2
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = .2

        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            self.BlockBlobDetector = cv.SimpleBlobDetector(params)
        else : 
            self.BlockBlobDetector = cv.SimpleBlobDetector_create(params)
    
        params.minArea = 700
        params.maxArea = 4200
    
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = .3
        params.maxCircularity = .70
    
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            self.BlockSurrBlobDetector = cv.SimpleBlobDetector(params)
        else : 
            self.BlockSurrBlobDetector = cv.SimpleBlobDetector_create(params)
            
            
        ###NEURAL NETWORKING
        if not dataCollect:
            mappingNNFile = open(self.trainingPath + 'mapping.json', 'r')
            mappingLetterNNJSON = mappingNNFile.read()
            mappingNNFile.close()
            self.mappingLetterNN = model_from_json(mappingLetterNNJSON)
            self.mappingLetterNN.load_weights(self.trainingPath + 'mapping.h5')
            self.mappingLetterNN.compile(loss='categorical_cross_entropy', optimizer='sgd')
        
        
            droneNNFile = open(self.trainingPath + 'drone.json', 'r')
            droneLetterNNJSON = mappingNNFile.read()
            droneNNFile.close()
            self.droneLetterNN = model_from_json(mappingLetterNNJSON)
            self.droneLetterNN.load_weights(self.trainingPath + 'drone.h5')
            self.droneLetterNN.compile(loss='categorical_cross_entropy', optimizer='sgd')

    #END OF __init__()

    
    
    

    #Identifies corner posts and calculates orient and rotation speed
    #   Arguments:
    #       hsvImage - HSV Image from camera
    #       Map map - Current map object
    #   Returns:
    #       Map map - Updated map
    def orientIdent(self, hsvImage, map, time):
        mask = cv.inRange(hsvImage, self.LowerLED, self.UpperLED)
        inv_mask = cv.bitwise_not(mask)

        keypoints = self.LEDBlobDetector.detect(inv_mask)
        
        postPoints = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            if x > self.X_MIN and x < self.X_MAX:            #Limit edge warping
                #xSquare = x**2
                #if keyY >= (28.81 - .0149 * x + .000139 * xSquare) and keyY < (17.132 - .0258 * x + .0001652 * xSquare):
                #    postPoints.append([keyY, 1, x])
                #elif keyY < (9.124 - .03289 * x + .000171 * xSquare):
                #    postPoints.append([keyY, 2, x])
                #elif keyY < (3.3572 - .03111 * x + .0001493 * xSquare):
                #    postPoints.append([keyY, 3, x])
                index, c, inv_thetaX, dir, d = map.tower.postDifferentiate(x, y, map.posts)
                if self.EX_LOG:
                    self.Logging.write("Post Keypoint: Index - %d   Distance - %2.3f\n" % (index, d))
                if index != 0:
                    postPoints.append([y, index, c, inv_thetaX, dir])
        #END OF for
        
        yMin = 400
        index = -1
        c = 0
        inv_thetaX = 0
        dir = 0
        for postPoint in postPoints:
            if postPoint[0] < yMin:
                index = postPoint[1]
                yMin = postPoint[0]
                c = postPoint[2]
                inv_thetaX = postPoint[3]
                dir = postPoint[4]
        if index != -1:
            map.tower.postCalibrate(time, index, c, inv_thetaX, dir, map.posts)
        
        if self.EX_LOG:
            self.Logging.write("Orientation: %1.3f     Speed: %2.3f\n" % (map.tower.armOrient, map.tower.armSpeed))
        
        #END OF post identification
        return map
    #END OF orientIdent()

    

    # Identifies all objects for the mapping phase
    #   Arguments:
    #       hsvImage - HSV Image from camera
    #       Map map - Current map object
    #       float time - Current time
    #   Returns:
    #       Map map - Updated map object
    def mappingIdent(self, hsvImage, map, time):  
        map.tower.recalculateArm(time)
        map.tower.setEyePos()
        
        #Identify posts
        mask = cv.inRange(hsvImage, self.LowerLED, self.UpperLED)
        inv_mask = cv.bitwise_not(mask)

        keypoints = self.LEDBlobDetector.detect(inv_mask)
        
        postPoints = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            if x > self.X_MIN and x < self.X_MAX:           #Limit edge warping
                #xSquare = x**2
                #if keyY >= (28.81 - .0149 * x + .000139 * xSquare) and keyY < (17.132 - .0258 * x + .0001652 * xSquare):
                #    postPoints.append([keyY, 1, x])
                #elif keyY < (9.124 - .03289 * x + .000171 * xSquare):
                #    postPoints.append([keyY, 2, x])
                #elif keyY < (3.3572 - .03111 * x + .0001493 * xSquare):
                #    postPoints.append([keyY, 3, x])
                index, c, inv_thetaX, dir, d = map.tower.postDifferentiate(x, y, map.posts)
                if index != 0:
                    postPoints.append([y, index, c, inv_thetaX, dir])
        #END OF for
        
        yMin = 400
        inv_thetaX = 0
        dir = 0
        c = 0
        
        for postPoint in postPoints:
            if postPoint[0] < yMin:
                yMin = postPoint[0]
                inv_thetaX = postPoint[3]
                dir = postPoint[4]
                c = postPoint[2]
        if dir != 0:
            map.tower.postRecalibrate(time, c, inv_thetaX, dir, map.posts)
            map.tower.setEyePos()
            
        if self.EX_LOG:   
            self.Logging.write("Orientation: %1.3f     Speed: %2.3f\n" % (map.tower.armOrient, map.tower.armSpeed))
        
        #Identify Drones
        droneIdent = False
        
        for drone in map.drones:
            if drone.point is None:
                droneIdent = True
                break
        
        if droneIdent:
            dronePoints = []
            for keypoint in keypoints:
                x, y = keypoint.pt
                if x > self.X_MIN and x < self.X_MAX: 
                    dronePoints.append(map.tower.mapPixel(x, y, map.DRONE_HEIGHT))
        
            tempDrones = []
            
            for point in dronePoints:
                backPoints = []
                tempPoints = dronePoints[:]
                tempPoints.remove(point)
                for tempPoint in tempPoints:
                    dist = abs(euclideanPoints(tempPoint, point) - self.DRONE_BACK_FRONT)
                    if dist < self.Ident_Threshold:
                        backPoints.append(tempPoint)
                if len(backPoints) > 1:
                    exit = False
                    for backPoint in backPoints:
                        if exit:
                            break
                        if len(backPoints) > 0:
                            tempBackPoints = backPoints
                            tempBackPoints.remove(backPoint)
                            for tempBack in tempBackPoints:
                                if exit:
                                    break
                                dist = abs(euclideanPoints(backPoint, tempBack) - self.DRONE_BACK_BACK)
                                if dist < self.Ident_Threshold:
                                    dx = point.x - (tempBack.x + backPoint.x) * .5
                                    dy = point.y - (tempBack.y + backPoint.y) * .5
                                    orient = arctan(dy, dx)
                                    location = Point(point.x - math.sin(orient) * self.DRONE_FRONT_CENTER, point.y - math.cos(orient) * self.DRONE_FRONT_CENTER)
                                    tempDrone.append([location, orient])
                                    exit = True
                                    break
            #END OF for
        
            for drone in tempDrones:
                dist = 1000
                for idx in range(0, 4):
                    d = euclideanPoints(drone[0], map.tower.MappingStartingPos[idx])
                    if d < dist:
                        machineID = idx
                map.addDrone(drone[0], drone[1], machineID)
        #END OF for
        
        
        
        
        
        
        #Identify Mothership
        if map.mothership is None:
            MothershipMask = cv.inRange(hsvImage, self.LowerMothershipLED, self.UpperMothershipLED)
            inv_MothershipMask = cv.bitwise_not(MothershipMask)
        
            keypoints = self.LEDBlobDetector.detect(inv_MothershipMask)
            
            if len(keypoints) > 3:
                motherPoints = []
                for keypoint in keypoints:
                    x, y = keypoint.pt
                    if x > self.X_MIN and x < self.X_MAX: 
                        motherPoints.append(map.tower.mapPixel(x, y, map.MOTHERSHIP_HEIGHT))
            
                exit = False
                for point in motherPoints:
                    if exit: break
                    sidePoints = []
                    tempPoints = motherPoints
                    tempPoints.remove(point)
                    for tempPoint in tempPoints:
                        dist = abs(euclideanPoints(tempPoint, point) - self.MOTHERSHIP_LED_DIST)
                        if dist < self.Ident_Threshold:
                            sidePoints.append(tempPoint)
                    if len(sidePoints) > 1:
                        for sidePoint in sidePoints:
                            if exit: break
                            if len(sidePoints) > 1:
                                tempSidePoints = sidePoints
                                tempSidePoints.remove(sidePoint)
                                for tempSide in tempSidePoints:
                                    if exit: break
                                    dist = abs(euclideanPoints(backPoint, tempBack) - self.MOTHERSHIP_LED_DBL)
                                    if dist < self.Ident_Threshold:
                                        motherPoints.remove(point)
                                        motherPoints.remove(tempSide)
                                        motherPoints.remove(sidePoint)
                                        testPoint = motherPoints[0]
                                        vectorx = tempSide.x - point.x
                                        vectory = tempSide.y - point.y
                                        orient = arctan(vectory, vectorx)
                                        ctrOrient = Angles_add(orient, self.THREE_HALF_PI)
                                        testVectorx = testPoint.x - point.x
                                        testVectory = testPoint.y - point.y
                                        testOrient = arctan(testVectory, testVectorx)
                                        testDiff = smallestDifferencePos(testOrient, ctrOrient)
                                        if testDiff > self.HALF_PI:
                                            orient = Angles_add(orient, math.pi)
                                            ctrOrient = Angles_add(ctrOrient, math.pi)
                                        dx = math.sin(ctrOrient) * self.MOTHERSHIP_LED_CENTR
                                        dy = math.cos(ctrOrient) * self.MOTHERSHIP_LED_CENTR
                                        location = Point(point.x + dx, point.y + dy)
                                        map.mothership = Mothership(location, orient)
                                        verts, walls = map.mothership.expand(len(map.vertexMap))
                                        map.vertexMap.extend(verts)
                                        map.collisionMap.extend(walls)
                                        exit = True
                                        break
        #End of Mothership Ident
        
        
        
        
        #Identify Obstacles
        obstIdent = len(map.obstacles) <= map.totalObstacles
        blockIdent = len(map.blocks) <= map.totalBlocks
        
        if obstIdent or blockIdent:
            BlockObstacleMask = cv.inRange(hsvImage, self.LowerBlockObstacle, self.UpperBlockObstacle)
            inv_BlockObstacleMask = cv.bitwise_not(BlockObstacleMask)
        
        if obstIdent:
            keypoints = self.ObstacleBlobDetector.detect(inv_BlockObstacleMask)
            
            obstaclePoints = []
            for keypoint in keypoints:
                x, y = keypoint.pt
                if x > self.X_MIN and x < self.X_MAX: 
                    obstaclePoints.append(map.tower.mapPixel(x, y, map.OBSTACLE_HEIGHT))
                    
            if self.EX_LOG:
                self.Logging.write("Obstacle Points: %d\n" % len(obstaclePoints))
            
            for point in obstaclePoints:
                for obstacle in map.obstacles:
                    dist = euclideanPoints(obstacle.location, point)
                    if dist < self.Expected_Error:
                        break
                else:
                    outputObstacle = Obstacle(point)
                    map.obstacles.append(outputObstacle)
                    verts, walls = outputObstacle.expand(len(map.vertexMap))
                    map.vertexMap.extend(verts)
                    map.collisionMap.extend(walls)
        #End of obstacle ident
        
        
        
        
        
        
        #Identify Blocks
        if blockIdent:
            keypoints = self.BlockBlobDetector.detect(BlockObstacleMask)
            
            blockPoints = []
            for keypoint in keypoints:
                x, y = keypoint.pt
                if x > self.X_MIN and x < self.X_MAX: 
                    blockPoints.append([map.tower.mapPixel(x, y, map.BLOCK_TOP_HEIGHT), Point(x, y)])
                    
            BlockSurrMask = cv.inRange(hsvImage, self.LowerBlockSurr, self.UpperBlockSurr)
            
            keypoints = self.BlockSurrBlobDetector.detect(BlockSurrMask)
            
            blockSurrPoints = []
            for keypoint in keypoints:
                x, y = keypoint.pt
                if x > self.X_MIN and x < self.X_MAX: 
                    blockSurrPoints.append([map.tower.mapPixel(x, y, map.BLOCK_MID_HEIGHT), Point(x, y)])
            
            blocks = []
            for point in blockPoints:
                for surrPoint in blockSurrPoints:
                    dist = euclideanPoints(point[0], surrPoint[0])
                    if dist < self.Expected_Error:
                        blocks.append([point, surrPoint])
                        
            if self.EX_LOG:
                self.Logging.write("Block Points: %d\n" % len(blocks))
                        
            for block in blocks:
                for mapBlock in map.blocks:
                    dist = euclideanPoints(block[1][0], mapBlock.location)
                    if dist < self.Expected_Error:
                        break
                else:
                    #Find Corners
                    topFacePixel = block[1][1]
                    topFaceBottomX = topFacePixel.x-self.HALF_BLOCK_X
                    topFaceBottomY = topFacePixel.y-self.HALF_BLOCK_Y
                    topFace = BlockSurrMask[int(topFaceBottomY):int(topFacePixel.y+self.HALF_BLOCK_Y),
                        int(topFaceBottomX):int(topFacePixel.x+self.HALF_BLOCK_X)]
                    gray = np.float32(BlockSurrMask)
                    dst = cv.cornerHarris(gray,5,3,0.04)
                    
                    keypoints = np.argwhere(dst > 0.22 * dst.max())
                    keypoints = [cv.KeyPoint(x[1], x[0], 1) for x in keypoints]
                    
                    cornerPoints = []
                    for keypoint in keypoints:
                        x, y = keypoint.pt
                        cornerPoints.append(map.tower.mapPixel(x+topFaceBottomX, y+topFaceBottomY, map.BLOCK_TOP_HEIGHT))
                    
                    if len(cornerPoints) > 1:
                        testDist = 2 * map.BLOCK_CTR_CORNER
                        exit = False
                        for cornerPoint in cornerPoints:
                            if exit: break
                            tempPoints = cornerPoints
                            tempPoints.remove(cornerPoint)
                            for tempPoint in tempPoints:
                                if exit: break
                                dist = euclideanPoints(cornerPoint, tempPoint) - testDist
                                if dist < .05:
                                    #Find Orient
                                    dx = cornerPoint.x - tempPoint.x
                                    dy = cornerPoint.y - tempPoint.y
                                    orient = arctan(dy, dx)
                                    orient = Angles_add(orient, self.QUARTER_PI)
                    
                                    ###If y < VALUE, Find Top Face
                                    #If y > VALUE, Find side face using orient
                                    dTheta = math.pi
                                    bestOrient = orient
                                    for x in range(0, 4):
                                        theta = smallestDifferencePos(orient, map.tower.armOrient)
                                        if theta < dTheta:
                                            dTheta = theta
                                            bestOrient = orient
                                        orient = Angles_add(orient, self.HALF_PI)
                                    bestOrient = Angles_add(bestOrient, math.pi)
                                    dx = math.sin(bestOrient) * .75
                                    dy = math.cos(bestOrient) * .75
                                    center = block[1][0]
                                    facePoint = Point(center.x + dx, center.y + dy)
                                    facePixel = map.tower.unmapPixel(facePoint.x, facePoint.y, map.BLOCK_MID_HEIGHT)
                                    face = hsvImage[int(facePixel.y-self.HALF_BLOCK_Y):int(facePixel.y+self.HALF_BLOCK_Y), 
                                        int(facePixel.x-self.HALF_BLOCK_X):int(facePixel.x+self.HALF_BLOCK_X)]
                                    
                                    
                                    #Alter face to be flat
                                    bestOrient = Angles_add(bestOrient, self.QUARTER_PI)
                                    dx = math.sin(bestOrient) * map.BLOCK_CTR_CORNER
                                    dy = math.cos(bestOrient) * map.BLOCK_CTR_CORNER
                                    facePoint = Point(center.x + dx, center.y + dy)
                                    crnr1Pixel = map.tower.unmapPixel(facePoint.x, facePoint.y, map.BLOCK_MID_HEIGHT)
                                    bestOrient = addMaybeNeg(bestOrient, -self.HALF_PI)
                                    dx = math.sin(bestOrient) * map.BLOCK_CTR_CORNER
                                    dy = math.cos(bestOrient) * map.BLOCK_CTR_CORNER
                                    facePoint = Point(center.x + dx, center.y + dy)
                                    crnr2Pixel = map.tower.unmapPixel(facePoint.x, facePoint.y, map.BLOCK_MID_HEIGHT)
                                    angle = math.atan2(crnr1Pixel.y - crnr2Pixel.y, crnr1Pixel.x - crnr2Pixel.x)
                                    angle = -angle
                                    M = cv.getRotationMatrix2D((self.HALF_BLOCK_X, self.HALF_BLOCK_Y), angle * self.TO_DEGREE, 1)
                                    #face = cv.warpAffine(face, M, (self.BLOCK_FACE_X, self.BLOCK_FACE_Y))
                                 
                                    #NN Identify face
                                    if self.dataCollect:
                                        self.blocksIdentified += 1
                                        cv.imwrite(self.trainingPath + ("towerBlock%d.png" % self.blocksIdentified), face)
                                    else:
                                        faceInput = cv.inRange(face, self.LowerBlockFace, self.UpperBlockFace)
                                        output = self.mappingLetterNN.predict(faceInput)
                                        idx = -1
                                        max = 0
                                        for x in range(0, 6):
                                            if output[x] > max:
                                                max = output[x]
                                                idx = x
                                        letters = ['A', 'B', 'C', 'D', 'E', 'F']
                                        letter = letters[idx]
                                        outputBlock = Block(center, orient, letter)
                                        verts, walls = outputBlock.expand(len(map.vertexMap), len(map.collisionMap))
                                        map.vertexMap.extend(verts)
                                        map.collisionMap.extend(walls)
                                        map.blocks.append(outputBlock)
                                    exit = True
                                    break
        #End of Block Ident
        
        
        return map
    # END OF mappingIdent()





   

    # Identifies posts and drones for the execution phase
    #   Arguments:
    #       hsvImage - HSV Image from camera
    #       Map map - Current map object
    #       float time - Current time
    #   Returns:
    #       Map map - Updated map object
    def execIdent(self, hsvImage, map, time):  
        map.tower.recalculateArm(time)
        map.tower.setEyePos()
        
        #Identify posts
        mask = cv.inRange(hsvImage, self.LowerLED, self.UpperLED)
        inv_mask = cv.bitwise_not(mask)

        keypoints = self.LEDBlobDetector.detect(inv_mask)
        
        postPoints = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            if x > self.X_MIN and x < self.X_MAX:             #Limit edge warping
                #xSquare = x**2
                #if keyY >= (28.81 - .0149 * x + .000139 * xSquare) and keyY < (17.132 - .0258 * x + .0001652 * xSquare):
                #    postPoints.append([keyY, 1, x])
                #elif keyY < (9.124 - .03289 * x + .000171 * xSquare):
                #    postPoints.append([keyY, 2, x])
                #elif keyY < (3.3572 - .03111 * x + .0001493 * xSquare):
                #    postPoints.append([keyY, 3, x])
                index, c, inv_thetaX, dir, d = map.tower.postDifferentiate(x, y, map.posts)
                if index != 0:
                    postPoints.append([y, index, c, inv_thetaX, dir])
        #END OF for
        
        yMin = 400
        inv_thetaX = 0
        dir = 0
        c = 0
        for postPoint in postPoints:
            if postPoint[0] < yMin:
                yMin = postPoint[0]
                inv_thetaX = postPoint[3]
                dir = postPoint[4]
                c = postPoint[2]
        if dir != 0:
            map.tower.postRecalibrate(time, c, inv_thetaX, dir, map.posts)
            map.tower.setEyePos()
            
            
        #Identify Drones
        dronePoints = []
        for keypoint in keypoints:
            x, y = keypoint.pt
            if x > self.X_MIN and x < self.X_MAX: 
                dronePoints.append(map.tower.mapPixel(x, y, map.DRONE_HEIGHT))
        
        tempDrones = []
        for point in dronePoints:
            backPoints = []
            tempPoints = dronePoints
            tempPoints.remove(point)
            for tempPoint in tempPoints:
                dist = abs(euclideanPoints(tempPoint, point) - self.DRONE_BACK_FRONT)
                if dist < self.Ident_Threshold:
                    backPoints.append(tempPoint)
            if len(backPoints) > 1:
                exit = False
                for backPoint in backPoints:
                    if exit:
                        break
                    if len(backPoints) > 0:
                        tempBackPoints = backPoints
                        tempBackPoints.remove(backPoint)
                        for tempBack in tempBackPoints:
                            if exit:
                                break
                            dist = abs(euclideanPoints(backPoint, tempBack) - self.DRONE_BACK_BACK)
                            if dist < self.Ident_Threshold:
                                dx = point.x - (tempBack.x + backPoint.x) * .5
                                dy = point.y - (tempBack.y + backPoint.y) * .5
                                orient = arctan(dy, dx)
                                location = Point(point.x - math.sin(orient) * self.DRONE_FRONT_CENTER, point.y - math.cos(orient) * self.DRONE_FRONT_CENTER)
                                tempDrone.append([location, orient])
                                exit = True
        #END OF for
        
        
        for tempDrone in tempDrones:
            dist = 1000
            drone = None
            for mapDrone in map.drones:
                d = euclideanPoints(drone[0], map.tower.MappingStartingPos[idx])
                if d < dist:
                    drone = mapDrone
            if drone.recalculate == True:
                drone.point = tempDrone[0]
                drone.orient = tempDrone[1]
            elif map.checkDronePos(drone):
                map.destroyCollision(drone.num)
                serialCom.stopDrones(drone.num)
                drone.endTime += 40
        
        return map
    #END OF execIdent()



    
    
    
    
    #Identifies letter of mothership slot
    #   Arguments:
    #       String filename - name of file
    #   Returns:
    #       char letter - Identified letter
    def identifyDroneImage(self, filename):
        image = cv.imread(filename)
        mask = cv.inRange(image, self.LowerBlockFace, self.UpperBlockFace)
        output = self.droneLetterNN.predict(mask)
        if output[0] > output[1]:
            return 'B'
        else:
            return 'E'
    #END OF identifyDroneImage()



#END OF Identification