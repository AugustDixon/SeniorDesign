#SearchTree Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

from .Node import *


def eval(e):
    return e.h

class SearchTree:
    #Constructor
    #   Arguments:
    #       Vertex h - Starting vertex
    #       int List destIdxList - List of possible destination points
    #       Point destination - Destination object centerpoint
    #       Vertex List vtxList - Vertex Map
    def __init__(self, h, destIdx, destination, vtxList):
        self.head = Node(None, h, destination, 0)
        self.destIdx = destIdxList
        self.dest = destination
        self.stack = []
        self.stack.append(head)
        self.vertexList = vtxList
    #END OF __init__()


    #Expand node function used in pathfind()
    def expand(self, node):
        edges = node.vertex.edges
        for edge in edges:
            self.stack.append(Node(node, self.vertexList[edge.destIdx], self.destination, edge.dist))
    #END OF expand()

    #Search function
    #   Return - int List - Path consisting of vertex indexes, not including current vertex
    def pathfind(self):
        node = self.stack.pop(0)
        if(node.vertex.idx in destIdx):
            path = []
            while(not node.parent is None):
                path.append(node.vertex.idx)
                node = node.parent
            path.reverse()
            return path
        node.expand()
        self.stack.sort(key=eval)
        return pathfind()
    #END OF pathfind()
#END OF SearchTree