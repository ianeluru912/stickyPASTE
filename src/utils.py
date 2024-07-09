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



def near_multiple(num, base=0.12, tolerance=1e-6):
    # Encuentra el múltiplo más cercano de la base
    closestMultiple = round(num / base) * base
    # Verifica si la diferencia es menor que la tolerancia
    return abs(num - closestMultiple) < tolerance