#Vertex Class definition
#Author: August Dixon
#LSU Senior Design 2018-2019 Team #72


#Point class to represent a 2D coordinate
class Point:
    #Constructor
    #   Arguments:
    #       double xx - x-coordinate
    #       double yy - y-coordinate
    def __init__(self, xx = 0, yy = 0):
        self.x = xx
        self.y = yy
    #END OF __init__

    #Checks for equality
    #   Arguments:
    #       Point p
    #   Returns:
    #       boolean result
    def equals(self, p):
        return (self.x == p.x) and (self.y == p.y)
    #END OF equals()
#END OF Point