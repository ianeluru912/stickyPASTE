import math
from point import Point
import numpy as np

def angle_diff(a, b):
    clockwise = (a - b) % math.tau
    counterclockwise = (b - a) % math.tau
    return min(clockwise, counterclockwise)

def targetPoint(position, angle, distance):
    angle+=math.pi/2
    x = position.x + distance * math.cos(angle)
    y = position.y - distance * math.sin(angle)
    return Point(x, y)

def puntoEnBordeY(origRobot, puntoAAveriguar):
    aux=puntoAAveriguar.y-(origRobot.y-0.06)
    return near_multiple(aux, 0.12, 0.0025)
 
def puntoEnBordeX(origRobot, puntoAAveriguar):
    aux=puntoAAveriguar.x-(origRobot.x-0.06)
    return near_multiple(aux, 0.12, 0.0025)

def near_multiple(num, base=0.12, tolerance=1e-6):
    # Encuentra el múltiplo más cercano de la base
    closestMultiple = round(num / base) * base
    # Verifica si la diferencia es menor que la tolerancia
    return abs(num - closestMultiple) < tolerance

def sortCw(listPoints):
    listAux=listPoints.copy()
    # Ordena los puntos en sentido horario
    aux=[]
    sumMin=1000
    primero=0
    for i in range(4):
        sumPoints=listAux[i][0]+listAux[i][1]
        if sumPoints<sumMin:
            sumMin=sumPoints
            primero=i
    # remove an element of numpy array
    pos1=listAux[primero]
    listAux=np.delete(listAux, primero, axis=0)

    sumMax=-10000
    tercero=0
    for i in range(3):
        sumPoints=listAux[i][0]+listAux[i][1]
        if sumPoints>sumMax:
            sumMax=sumPoints
            tercero=i
    pos3=listAux[tercero]
    listAux=np.delete(listAux, tercero, axis=0)

    if listAux[0][0]>listAux[1][0]:
        pos2=listAux[0]
        pos4=listAux[1]
    else:
        pos2=listAux[1]
        pos4=listAux[0]
    return np.array([pos1, pos2, pos3, pos4],dtype=np.float32)

def normalizacion_radianes(radianes):
    if radianes > math.pi:
        radianes -= math.pi*2
        return radianes
    elif radianes < -math.pi:
        radianes += math.pi*2
        return radianes
    return radianes