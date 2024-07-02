from point import Point
import math

class Rectangle:
    def __init__(self, top, left, bottom, right):
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right

        self.topLeft = Point(self.left, self.top)
        self.bottomRight = Point(self.right, self.bottom)

    def intersects(self, rect):
        a = self
        b = rect
        if b.left > a.right: return False
        if b.top > a.bottom: return False
        if b.right < a.left: return False
        if b.bottom < a.top: return False
        return True    
    