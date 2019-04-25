#Main Tower Robot Raspberry Pi Script
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

#import project files
from tower import *


import sys
import os
import shutil
import numpy as np
import cv2 as cv
import time as tiempo
import datetime

############ Constants, Globals, and Options ###############

#####Constants and Globals
DATA_COLLECTION_TIME = 30            #Time in seconds that the camera will collect data



global time, StartTime, LogFile, map, camera, serialCom, ident


#####Options
DEBUG = True        #Enables debug mode
EX_LOG = True       #Extends the information stored in the debug log

MODE = 5
#Execution mode
#   1 = Competition mode (4 Drone)
#   2 = Competition mode (2 Drone)
#   3 = Data Collection mode
#   4 = Drone camera capture mode
#   5 = Late Data Collection mode

ROUND = 1
#Round number
#   1 = 2 Blocks, 5 Obstacles
#   2 = 4 Blocks, 10 Obstacles
#   3 = 6 Blocks, 15 Obstacles







################### Helper Functions ########################

#Stops Execution early
def breakCode():
    cleanup()
    sys.exit()
#END OF breakCode()

#Deconstructs and closes objects as necessary
def cleanup():
    global time, StartTime, camera, serialCom
    if not serialCom is None:
        serialCom.sendStop()
    if not camera is None or not camera.isOpened():
        camera.release()
    tdiff = tiempo.perf_counter() - StartTime
    time = tiempo.perf_counter()
    LogFile.write("Start Time: %7.3f    End Time: %7.3f     Total Runtime: %7.3f\n" % (StartTime, time, tdiff))
    LogFile.close()
    #TODO as necessary
#END OF cleanup()


############# Drone Camera Capture Function ##################
def droneCap():
    global time, LogFile, map, serialCom
    
    captureInst = []
    captureInst.append(map.drones[0].createInstruction(b'M', 100, 2000, b'N'))
    captureInst.append(map.drones[0].createInstruction(b'T', 1, 2000, b'N'))
    captureInst.append(map.drones[0].createInstruction(b'T', -1, 2000, b'N'))
    captureInst.append(map.drones[0].createInstruction(b'F', 0, 0, b'E'))
    serialCom.transmitInstructions('1', captureInst)
    while True:
        succ, mess = serialCom.checkMessage()
        
        if succ:
            print(mess)
            if mess == "B":
                if (DEBUG):
                    temp = tiempo.perf_counter()
                    tdiff = temp - time
                    time = temp
                    LogFile.write("Time: %7.3f    Time Diff: %7.3f    Stop Signal From Arduino\n" % (time, tdiff))
                breakCode()
            elif mess == "L":
                filename = serialCom.receiveImage()
                if (DEBUG):
                    temp = tiempo.perf_counter()
                    tdiff = temp - time
                    time = temp
                    LogFile.write("Time: %7.3f    Time Diff: %7.3f    Image Received\n" % (time, tdiff))
                break
    #END OF while
    breakCode()
    
#END OF droneCap()




################ Data Collection Function ####################
#   Arguments:
#       camera - VideoCapture object for the camera
def dataCollect(camera):
    global time, LogFile

    cwd = "/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower"
    #Remove old data and create /data directory
    if(os.path.isdir(cwd + "/data")):
        try:
            shutil.rmtree(cwd + "/data")
        except:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Old data removal failed, breaking\n" % (time, tdiff))
                breakCode()
    try:
        os.mkdir(cwd + "/data")
    except OSError:
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: /data directory creation failed, breaking\n" % (time, tdiff))
            breakCode()
    frameNum = 1

    #Collect frames and slices for DATA_COLLECTION_TIME seconds
    dataStart = tiempo.perf_counter()
    endTime = DATA_COLLECTION_TIME + dataStart
    while (tiempo.perf_counter() < endTime):
        if (DEBUG):
            frameStart = tiempo.perf_counter()
            tdiff = frameStart - time
            time = frameStart
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning frame %d\n" % (
                time, tdiff, frameNum))


        ret, image = camera.read()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()


        #Save original image
        ret = cv.imwrite(cwd + "/data/frame%d.png" % frameNum, image)
        if (not ret):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to write image, breaking\n" % (
                time, tdiff))
            breakCode()


        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - frameStart
            time = temp
            LogFile.write("Time: %7.3f    Frame Time: %7.3f    Frame %d finished\n" % (
                time, tdiff, frameNum))
        frameNum += 1
    #END OF while()
    
    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Data Collection Completed\n" % (time, tdiff))
    breakCode()
#END OF dataCollect()





################ Standard pipeline function ##################
#   Arguments:
#       camera - VideoCapture object for the camera
def pipeline(camera):
    global time, LogFile, map, ident

    frameNum = 0
    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Orientation Phase\n" % (time, tdiff))

    #Orientation Phase
    map.tower.armCalibrate = False
    map.tower.oldOrient = map.tower.oldTime = None
    while(True):
        frameNum += 1
        #Grab frame from camera
        frameCollectTime = tiempo.perf_counter()
        ret, image = camera.read()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()

        #Identify Posts
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        map = ident.orientIdent(hsvImage, map, frameCollectTime)


        #Break if calibrated
        if map.tower.armCalibrate:
            break
    #END OF while




    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Mapping Phase\n" % (time, tdiff))

    #Mapping Phase
    timeout = tiempo.perf_counter() + 30
    while(True):
        frameNum += 1
        # Grab frame from camera
        frameCollectTime = tiempo.perf_counter()
        ret, image = camera.read()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()

        ###Preprocess
        #Convert to HSV for easier masking
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        
        map = ident.mappingIdent(hsvImage, map, time)
        
                
        if map.checkMapping():
            break
        if tiempo.perf_counter() > timeout:
            break
    #END OF while

    
    
    
    #Map Expansion
    for v in map.vertexMap:
        v.expandEdges(map.vertexMap, map.collisionMap)

    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Execution Phase\n" % (time, tdiff))

    map.executionPrep()                                                                     
    map.pathfind(SerialCom)                                                                          
        
        
        
    #Execution Phase
    while(True):
        frameNum += 1
        # Grab frame from camera
        frameCollectTime = tiempo.perf_counter()
        ret, image = camera.read()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()

        #Convert to HSV for easier masking
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        #Calculate expected Drone positions
        map.calculateDronePos(frameCollectTime)
        
        #Identify Drones
        map = ident.execIdent(hsvImage, map, frameCollectTime)
        
        map.recalculate(serialCom)
        
        
        succ, mess = serialCom.checkMessage()
        
        if succ:
            if mess == "B":
                if (DEBUG):
                    temp = tiempo.perf_counter()
                    tdiff = temp - time
                    time = temp
                    LogFile.write("Time: %7.3f    Time Diff: %7.3f    Stop Signal From Arduino\n" % (time, tdiff))
                breakCode()
            elif mess == "L":
                filename = serialCom.receiveImage()
                letter = ident.identifyDroneImage(filename)                                           
                map.mothershipIdentLetter(letter, serialCom)

        map.checkDroneStatus(serialCom)                                                      
    #END OF while


#END OF pipeline()



############## Late Data Collection Function ##############
#   Arguments:
#       camera - VideoCapture object for the camera
def dataPipeline(camera):
    global time, LogFile, map, ident
    
    cwd = "/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower"
    #Remove old data and create /data directory
    if(os.path.isdir(cwd + "/data")):
        try:
            shutil.rmtree(cwd + "/data")
        except:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Old data removal failed, breaking\n" % (time, tdiff))
                breakCode()
    try:
        os.mkdir(cwd + "/data")
    except OSError:
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: /data directory creation failed, breaking\n" % (time, tdiff))
            breakCode()
    
    frameNum = 0
    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Orientation Phase\n" % (time, tdiff))

    #Orientation Phase
    map.tower.armCalibrate = False
    map.tower.oldOrient = map.tower.oldTime = None
    timeout = tiempo.perf_counter() + 30
    while(True):
        frameNum += 1
        #Grab frame from camera
        frameCollectTime = tiempo.perf_counter()
        for i in range(0, 5):
            camera.grab()
        ret, image = camera.retrieve()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()
            
        
        ret = cv.imwrite(cwd + "/data/frame%d.png" % frameNum, image)
        if (not ret):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to write image, breaking\n" % (
                time, tdiff))
            breakCode()

        #Identify Posts
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        map = ident.orientIdent(hsvImage, map, frameCollectTime)

        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - frameCollectTime
            time = temp
            LogFile.write("Time: %7.3f    Frame Time: %7.3f    Frame %d finished\n" % (
                time, tdiff, frameNum))
                
        #Break if calibrated
        if map.tower.armCalibrate:
            break
        if tiempo.perf_counter() > timeout:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Orientation timeout\n" % (time, tdiff))
            breakCode()
    #END OF while




    if (DEBUG):
        temp = tiempo.perf_counter()
        tdiff = temp - time
        time = temp
        LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Mapping Phase\n" % (time, tdiff))

    #Mapping Phase
    timeout = tiempo.perf_counter() + 30
    while(True):
        frameNum += 1
        # Grab frame from camera
        frameCollectTime = tiempo.perf_counter()
        for i in range(0, 5):
            camera.grab()
        ret, image = camera.retrieve()
        if not ret:
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to retrieve frame %d, breaking\n" % (
                time, tdiff, frameNum))
            breakCode()
        
        ret = cv.imwrite(cwd + "/data/frame%d.png" % frameNum, image)
        if (not ret):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failed to write image, breaking\n" % (
                time, tdiff))
            breakCode()

        ###Preprocess
        #Convert to HSV for easier masking
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        map = ident.mappingIdent(hsvImage, map, time)
        
        if EX_LOG:
            if map.mothership is None:
                LogFile.write("Mothership Not Identified\n")
            else:
                LogFile.write("Mothership: (%2.2f, %2.2f) Orient: %2.3f\n" % (map.mothership.location.x, map.mothership.location.y, map.mothership.orientation))
            LogFile.write("Obstacles: %d\n" % len(map.obstacles))
            for obstacle in map.obstacles:
                LogFile.write("(%2.2f, %2.2f)\n" % (obstacle.location.x, obstacle.location.y))
            LogFile.write("Blocks Identified: %d\n" % ident.blocksIdentified)
                
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - frameCollectTime
            time = temp
            LogFile.write("Time: %7.3f    Frame Time: %7.3f    Frame %d finished\n" % (time, tdiff, frameNum))
                
        if map.checkMapping():
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Mapping Finished\n" % (time, tdiff))
            break
        if tiempo.perf_counter() > timeout:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Mapping finished by timeout\n" % (time, tdiff))
            break
    #END OF while
    
    if DEBUG:
        if map.mothership is None:
            LogFile.write("Mothership Not Identified\n")
        else:
            LogFile.write("Mothership: (%2.2f, %2.2f) Orient: %2.3f\n" % (map.mothership.location.x, map.mothership.location.y, map.mothership.orientation))
        LogFile.write("Obstacles: %d\n" % len(map.obstacles))
        for obstacle in map.obstacles:
            LogFile.write("(%2.2f, %2.2f)\n" % (obstacle.location.x, obstacle.location.y))
        LogFile.write("Blocks Identified: %d\n" % ident.blocksIdentified)
    breakCode()

#END OF dataPipeline()




###################### Main Function #######################
def main():
    #######
    # Programming initializations
    global time, StartTime, LogFile, map, camera, serialCom, ident
    serialCom = camera = ident = None
    time = tiempo.perf_counter()
    StartTime = time
    LogFile = open("/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/Log.txt", "w")
    if (DEBUG):
        LogFile.write("Internal Time: %7.3f    Beginning of program  %s\n" % (time, datetime.datetime.now()))

    #Instantiate map
    map = Map.Map(ROUND)
    #TODO: Unknown currently


    try:
        #######
        # Initialization Hardware checks:
        #   1. Check for and start camera
        #   2. Wait for and establish communication with Arduino
        #   3. Collect number and identification of drone robots
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning of Hardware Checks\n" % (time, tdiff))

        #Check for and initialize camera
        camera = cv.VideoCapture(0)
        if camera is None or not camera.isOpened():
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Unable to connect to camera, Breaking\n" % (time, tdiff))
            breakCode()
        else:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Camera Connected\n" % (time, tdiff))
                
                
        if MODE == 5 or MODE == 4 or MODE == 3:
            ident = Identification.Identification(True, LogFile, EX_LOG)
        else:
            ident = Identification.Identification(False, LogFile, EX_LOG)

        #Establish Communication with Arduino
        serialCom = Communication.SerialCom()
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    Begin waiting for Arduino communication test\n" % (time, tdiff))
        if(serialCom.commTestWAIT(MODE)):
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Communication test success\n" % (time, tdiff))
        else:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Communication test failure, Breaking\n" % (time, tdiff))
            breakCode()

        #Receive drone data from Arduino
        if (MODE == 1):
            map.drones = serialCom.droneConnectWAIT(4)
        elif (MODE == 2):
            map.drones = serialCom.droneConnectWAIT(2)
        elif (MODE == 4):
            map.drones = serialCom.droneConnectWAIT(1)



        #######
        # Wait on Start signal from Arduino MEGA, will occur after Arduino has raised arm and begun rotation
        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    Awaiting Start Signal\n" % (time, tdiff))
        if (serialCom.startWAIT()):
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Start Signal Recieved\n" % (time, tdiff))
        else:
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    ERROR: Failure to receive start signal, Breaking\n" % (time, tdiff))
            breakCode()

        #######
        # Check for special modes
        # 1 = Competition Mode (4 Drone)
        # 2 = Competition Mode (2 Drone)
        # 3 = Data Collection mode
        # 4 = Drone Camera Capture mode
        # 5 = Late Data Collection mode
        if (MODE == 3):
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Data Collection\n" % (time, tdiff))
            dataCollect(camera)
            breakCode()
        if (MODE == 4):
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Drone Camera Capture\n" % (time, tdiff))
            droneCap()
            breakcode()
        if (MODE == 5):
            if (DEBUG):
                temp = tiempo.perf_counter()
                tdiff = temp - time
                time = temp
                LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Data Collection\n" % (time, tdiff))
            dataPipeline(camera)
            breakCode()

        if (DEBUG):
            temp = tiempo.perf_counter()
            tdiff = temp - time
            time = temp
            LogFile.write("Time: %7.3f    Time Diff: %7.3f    Beginning Competition Functions\n" % (time, tdiff))

        #######
        # Perform Standard pipeline
        pipeline(camera)
    except Exception as e:
        LogFile.write("ERROR: " + str(e) + "\n")
    finally:
        pass

    #######
    # Finishing procedures:
    cleanup()
    sys.exit()
#END OF main()


if __name__ == "__main__":
    main()