import math
from point import Point

def angle_diff(a, b):
    clockwise = (a - b) % math.tau
    counterclockwise = (b - a) % math.tau
    return min(clockwise, counterclockwise)

def targetPoint(position, angle, distance):
    angle+=math.pi/2
    x = position.x + distance * math.cos(angle)
    y = position.y - distance * math.sin(angle)
    return Point(x, y)