#Node Class definition
#This class is used in A* search pathfinding
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

import math

#Constant cost of making turns
COSTCONST = 7

class Node:
    #Constructor
    #   Arguments:
    #       Node p - Parent Node
    #       Vertex vert - Node vertex
    #       Point d - Destination simple point
    #       double distance - Edge distance from parent
    def __init__(self, p, vert, d, distance):
        self.parent = p
        self.vertex = vert
        self.g = p.g + distance + COSTCONST
        self.h = math.sqrt((vert.p.x-d.x)**2 + (vert.p.y-d.y)**2) + self.g
    #END OF __init__()


#END OF Node