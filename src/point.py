import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, point):
        dx = self.x-point.x
        dy = self.y-point.y
        return math.sqrt(dx**2 + dy**2)
    
    def __str__(self) -> str:
        return f"({self.x:.3f}, {self.y:.3f})"