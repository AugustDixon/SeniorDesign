#SerialCom class definition file
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

import serial
import time
import struct
from .Obstacles.Drone import *

#Message Wrappers
#   <O>/Start Message:
#       's' - Message Start
#   <S>/Source & <D>/Destination:
#       'p' - Raspberry Pi
#       'a' - Arduino Mega
#       '1', '2', '3', '4' - Drone Robots
#       '?' - Unknown Drone Robot
#       'd' - All Drone Robots (Destination Only)
#       'A' - ALL (Destination Only)
#   <M>/Message:
#       'A' - Acknowledgement response - Anywhere
#       'T' - Test signal - commTestWAIT()
#       'S' - Round start signal - startWAIT()
#       'B' - Stop signal
#       'C' - Drone Connect
#       'x' - Silence command
#       'u' - Stop Silence
#       "I#" - Instruction sending start (Requires channel silence)
#       'L' - Letter image sending start (Requires channel silence)
#   <X>/End Message:
#       'x' - Terminator
#
#   <O><S><D><M><X>

#Instruction Format - 10 Bytes
#   <O>/Operation:
#       'M' - Move - 2 arg
#       'T' - Turn - 2 arg
#       'F' - Activate Light - 0 arg
#       'L' - Lift Block Sequence - 0 arg
#       'D' - Deposit Block Sequence - 0 arg
#       'C' - Camera Capture Sequence - 0 arg
#       'B' - Stop command - 0 arg
#   <Arg1>/Argument 1 - 2 Byte Int, Motor Setting usually
#   <Arg2>/Argument 2 - 4 Byte Long, Time in milliseconds usually
#   <S>/Special:
#       'E' - End of instruction set
#       'N' - Default
#
#   <O><Arg1><Arg2><S>

#SerialCom class to wrap serial communications between Raspberry Pi and Arduino Mega
class SerialCom:
    #Constructor
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=None)
        self.ser.reset_input_buffer()
    #END OF __init__()

    #Ignores Instruction transmission
    def ignoreInstruction(self):
        while(true):
            temp = self.ser.read(8);
            if(temp[7] == b'E'):
                break
        #END OF while
    #END OF ignoreInstruction()

    #Transmits instruction list
    #   Arguments:
    #       char destination - Drone destination
    #       Instruction[] instructions - Instruction List
    def transmitInstructions(self, destination, instructions):
        self.ser.write(bytes("spaI%sx" % destination, 'utf-8'))
        for instruct in instructions:
            output = bytearray(8)
            output[0] = list(instruct.O)[0]
            arg1 = struct.pack(">h", instruct.Arg1)
            output[1] = arg1[0]
            output[2] = arg1[1]
            arg2 = struct.pack(">l", instruct.Arg2)
            output[3] = arg2[0]
            output[4] = arg2[1]
            output[5] = arg2[2]
            output[6] = arg2[3]
            output[7] = list(instruct.S)[0]
            print(output)
            self.ser.write(output)
    #END OF transmitInstructions()
    
    #Sends stop instruction to all drones
    #   Arguments:
    #       char dest - Destination
    def stopDrones(self, dest):
        self.ser.write(bytes("spaI%cx" % dest, 'utf-8'))
        output = bytearray(8)
        output[0] = b'B'
        for x in range(1, 7):
            output[x] = 0x00
        output[7] = b'E'
        self.ser.write(output)
    #END OF stopDrones()


    #Read message while blocking
    #   Return - String message
    def readMessage(self):
        while True:
            character = self.ser.read().decode("utf-8")
            if (character == 's'):
                input = self.ser.read_until(b'x').decode("utf-8")
                if (input[1] == 'p' or input[1] == 'A'):
                    message = input[2:-1]
                    return message
                elif (input[2] == 'I'):
                    self.ignoreInstruction();
    #END OF readMessage()
    
    #Read message without blocking
    #   Returns:
    #       boolean succ - success
    #       String message
    def checkMessage(self):
        if(self.ser.in_waiting > 0):
            while(self.ser.in_waiting > 0):
                character = self.ser.read().decode("utf-8")
                if (character == 's'):
                    input = self.ser.read_until(b'x').decode("utf-8")
                    if (input[1] == 'p' or input[1] == 'A'):
                        return True, input[2:-1]
                    elif (input[2] == 'I'):
                        self.ignoreInstruction();
                        return False, ""
            else:
                return False, ""
        else:
            return False, ""
    #END OF checkMessage()


    # Receives JPEG Image
    #   Return - String Filename
    def receiveImage(self):
        filename = '/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/DroneImage.jpg'
        size = int(self.readMessage())
        file = open(filename, 'wb')
        image = self.ser.read(size)
        file.write(image)
        file.close()
        return filename
    # END OF receiveImage()


    #Test communications
    #Waits on Arduino test comm and responds
    #   Arguments:
    #       mode - integer execution mode option
    #   Return - boolean result
    def commTestWAIT(self, mode):
        input = self.readMessage()
        if (input != "T"):
            return False
        else:
            self.ser.write(bytes("spa%dx" % mode, 'utf-8'))
            return True
    #END OF commTestWAIT()


    #Helper for droneConnectWAIT()
    #   Return - Drone - Drone data
    def droneConnect(self):
        input = self.readMessage()
        return Drone(input[0], ord(input[1]))
    #END OF droneConnect()


    #Waits for Arduino Mega to connect to drones
    def droneConnectWAIT(self, numDrones):
        drones = []
        for i in range(0, numDrones):
            drones.append(self.droneConnect())
            self.ser.write(bytes("spaAx", 'utf-8'))
        return drones
    #END OF commTestWAIT()


    #Waits on start signal
    #   Return - boolean result
    def startWAIT(self):
        input = self.readMessage()
        print(input)
        if (input != "S"):
            print(input)
            return False
        else:
            #self.ser.write(bytes('spaAx', 'utf-8'))
            return True
    #END OF startWAIT()


    #Sends Stop signal to Arduino
    def sendStop(self):
        self.ser.write(bytes("spaBx", 'utf-8'))
    #END OF sendStop()


#END OF SerialCom