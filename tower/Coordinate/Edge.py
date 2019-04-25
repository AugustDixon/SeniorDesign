#Edge Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72

#Class Edge contains an edge from one vertex to another, visible vertex
class Edge:
    # Constructor
    #   Arguments:
    #       int bb - Destination Vertex idx
    #       double d - Distance between vertices
    def __init__(self, bb, d):
        self.destIdx = bb.idx
        self.dist = d
    #END OF __init__()

#END OF Wall