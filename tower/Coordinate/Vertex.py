#Vertex Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

import math
from .Edge import *

#Collision optimization scratchwork
#def ccw(A,B,C):
#    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
#
#def ccw(A,B,C):
#    return CyAy * BxAx > ByAy * CxAx
#
#Return true if line segments AB and CD intersect
#def intersect(A,B,C,D):
#    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

#Intersection function optimized for usage, BxAx and ByAy calculated ahead of time. DyAy, CxAx, CyAy, DxAx used multiple times
def intersect(A,B,C,D,BxAx,ByAy):
    DyAy = D.y-A.y
    CxAx = C.x-A.x
    CyAy = C.y-A.y
    DxAx = D.x-A.x
    return (DyAy * CxAx > CyAy * DxAx) != (D.y-B.y * C.x-B.x > C.y-B.y * D.x-B.x) and (CyAy * BxAx > ByAy * CxAx) != (DyAy * BxAx > ByAy * DxAx)
#END OF intersect()



#Vertex class to represent a point as well as the data associated with it
class Vertex:
    #Constructor
    #   Arguments:
    #       Point pp - Coordinate of Point
    #       int index - index of vertex on vertex map
    #       int List associatedIdx - List of associated vertex ids from the inciting object
    #       int List neighborIdx - Associated Idx which happen to be neighbors
    def __init__(self, pp, index, associatedIdx, neighborIdx):
        self.p = pp
        self.idx = index
        self.edges = []                 #Empty edge list
        self.assIdx = associatedIdx     #Contains associated vertices (From same object)
        self.neighIdx = neighborIdx     #Contains associated vertices which are neighbors
        self.expandedIdx = []           #Contains vertices which have already expanded to this vertex
    #END OF __init__()

    #Helper function for expandEdges()
    #   Arguments:
    #       int idx - index of calling vertex
    #       double dist - distance between vertices
    def addEdge(self, idx, dist):
        self.expandedIdx.append(idx)
        self.edges.append(Edge(idx, dist))
    #END OF addEdge()

    #Function to find all available edges from this vertex
    #   Arguments:
    #       Vertex List vertices - Full Board Vertex Map
    #       Wall List collision - Full Board Collision Map
    def expandEdges(self, vertices, collision):
        for x in range(0, len(vertices)):
            vertex = vertices[x]
            if(not (x in self.expandedIdx)):
                if(not (x in self.assIdx)):
                    BxAx = vertex.p.x - self.p.x
                    ByAy = vertex.p.y - self.p.y
                    for w in collision:
                        if(intersect(self.p, vertex.p, w.a, w.b, BxAx, ByAy)):
                            break
                    else:
                        dist = math.sqrt(BxAx ** 2 + ByAy ** 2)
                        self.edges.append(Edge(vertex.idx, dist))
                        vertices[x].addEdge(self.idx, dist)
                elif(x in self.neighIdx):
                    BxAx = vertex.p.x - self.p.x
                    ByAy = vertex.p.y - self.p.y
                    dist = math.sqrt(BxAx ** 2 + ByAy ** 2)
                    self.edges.append(Edge(vertex.idx, dist))
                    vertices[x].addEdge(self.idx, dist)
        #END OF for
    #END OF expandEdges()

    #Variant of expandEdges for drone vertices
    def droneExpansion(self, vertices, collision):
        for x in range(0, len(vertices)):
            vertex = vertices[x]
            BxAx = vertex.p.x - self.p.x
            ByAy = vertex.p.y - self.p.y
            for w in collision:
                if(intersect(self.p, vertex.p, w.a, w.b, BxAx, ByAy)):
                    break
                else:
                    dist = math.sqrt(BxAx ** 2 + ByAy ** 2)
                    self.edges.append(Edge(vertex.idx, dist))
    #END OF droneExpansion()
#END OF Vertex
