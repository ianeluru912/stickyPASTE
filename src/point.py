import math
class Point:
    @classmethod
    def from_angle(cls, angle, magnitude=1):
        y = magnitude * math.sin(angle)
        x = magnitude * math.cos(angle)
        return cls(x, y)
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def length(self):
        return math.sqrt(self.x*self.x + self.y*self.y)
    
    def dot(self, point):
        return self.x*point.x + self.y*point.y
    
    def angle_to(self, point):
        if point.length() == 0: return 0
        return math.acos(self.dot(point) / (self.length() * point.length()))

    def distance_to(self, point):
        dx = self.x-point.x
        dy = self.y-point.y
        return math.sqrt(dx**2 + dy**2)
    
    def __str__(self) -> str:
        return f"({self.x:.3f}, {self.y:.3f})"
    
    def distance_vector(self, target): # Target es a donde yo quiero llegar
        vector = Point(target.x - self.x, target.y - self.y)
        return vector 

    def distance_to_target(self, target):
        vector = self.distance_vector(target)
        return vector.length()