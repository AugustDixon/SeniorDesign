#Contains functions for angle operations
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

import math

TWO_PI = math.pi * 2

def smallestDifference(ang1, ang2):
    temp = ang1 - ang2
    if temp <= 0:
        dir = 1
    else:
        dir = -1
    temp = abs(temp)
    if temp > math.pi:
        temp = TWO_PI - temp
        dir = -dir
    return dir, temp
#END OF difference()

def smallestDifferencePos(ang1, ang2):
    temp = abs(ang1 - ang2)
    if temp > math.pi:
        return TWO_PI - temp
    else:
        return temp
#END OF difference()

def add(ang1, ang2):
    temp = ang1 + ang2
    if temp >= TWO_PI:
        temp -= TWO_PI
    return temp
#END OF add()

def addMaybeNeg(ang1, ang2):
    temp = ang1 + ang2
    if temp >= TWO_PI:
        temp -= TWO_PI
    elif temp < 0:
        temp += TWO_PI
    return temp
#END OF addMaybeNeg()

def arctan(y, x):
    return math.atan2(y,x) + math.pi
#END OF arctan()