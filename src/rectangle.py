
import math

class Rectangle:
    def __init__(self, top, bottom, left, right):
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right

        self.topLeft = (self.left, self.top)
        self.bottomRight = (self.right, self.bottom)

        self.vector = math.sqrt((self.topLeft - self.bottom)**2 + (self.bottomRight - self.left)**2)

    def rectangleUpright(self, tile):
        if self.top == tile.HEIGHT * 2 and self.right == tile.WIDTH:
            return self.topLeft, self.bottomRight

    def rectangleHorizontal(self, tile):
        if self.top == tile.HEIGHT and self.right == tile.WIDTH * 2:
            return self.topLeft, self.bottomRight


        
        



