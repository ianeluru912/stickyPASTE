from controller import Robot as WebotsRobot
from map import Map, Tile
from image import ImageProcessor
from point import Point
import math
import utils
import struct
import numpy as np

TIME_STEP = 16
MAX_VEL = 3.14  # Reduzco la velocidad para minimizar desvío

class Robot:
    def __init__(self):

        self.robot = WebotsRobot()
        self.emitter = self.robot.getDevice("emitter")
        self.wheelL = self.robot.getDevice("wheel1 motor")
        self.wheelL.setPosition(float("inf"))

        self.wheelR = self.robot.getDevice("wheel2 motor")
        self.wheelR.setPosition(float("inf"))

        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(TIME_STEP)

        self.inertialUnit = self.robot.getDevice("inertial_unit")
        self.inertialUnit.enable(TIME_STEP)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.colorSensor = self.robot.getDevice("colour_sensor")
        self.colorSensor.enable(TIME_STEP)

        self.camI = self.robot.getDevice("camaraIzquierda")
        self.camI.enable(TIME_STEP)

        self.camD = self.robot.getDevice("camaraDerecha")
        self.camD.enable(TIME_STEP)

        self.imageProcessor = ImageProcessor()
       
        self.position = None
        self.rotation = 0
        self.rangeImage = None
        self.posicion_inicial = None

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)
        self.step()

        self.posicion_inicial = self.position
        self.map = Map(self.posicion_inicial)

    def step(self):
        result = self.robot.step(TIME_STEP)
        self.updateVars()
        return result

    def delay(self, ms):
        initTime = self.robot.getTime()
        while self.step() != -1:
            if (self.robot.getTime() - initTime) * 1000.0 >= ms:
                break

    def updateVars(self):
        self.updatePosition()
        self.updateRotation()
        self.updateRangeImage()

    def updatePosition(self):
        x, _, y = self.gps.getValues()
        self.position = Point(x, y)

    def updateRotation(self):
        _, _, yaw = self.inertialUnit.getRollPitchYaw()
        self.rotation = self.normalizar_radianes(yaw % math.tau)

    def updateRangeImage(self):
        self.rangeImage = self.lidar.getRangeImage()[1024:1536]

    def girar(self, rad):
        lastRot = self.rotation
        deltaRot = 0

        while self.step() != -1:
            deltaRot += utils.angle_diff(self.rotation, lastRot)
            lastRot = self.rotation

            diff = utils.angle_diff(deltaRot, abs(rad))

            mul = (5/math.pi) * diff
            mul = min(max(mul, 0.05), 1)

            if rad > 0:
                self.wheelL.setVelocity(mul*MAX_VEL)
                self.wheelR.setVelocity(-mul*MAX_VEL)
            else:
                self.wheelL.setVelocity(-mul*MAX_VEL)
                self.wheelR.setVelocity(mul*MAX_VEL)

            if diff <= 0.005:
                break

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

    def avanzar(self, distance):
        initPos = self.position

        while self.step() != -1:
            diff = abs(distance) - initPos.distance_to(self.position)

            vel = min(max(diff/0.01, 0.1), 1)
            if distance < 0: vel *= -1

            self.wheelL.setVelocity(vel*MAX_VEL)
            self.wheelR.setVelocity(vel*MAX_VEL)

            if diff < 0.001:
                break

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

    def hayAlgoIzquierda(self):
        leftDist = self.rangeImage[128]
        if leftDist < 0.08:
            return True
        else:
            if self.imageProcessor.hay_posible_agujero(self.camI.getImage()):
                return True
            else:
                return False

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128*3]
        if rightDist < 0.08:
            return True
        else:
            if self.imageProcessor.hay_posible_agujero(self.camD.getImage()):
                return True
            else:
                return False


    def hayAlgoAdelante(self):
        frontDist = self.rangeImage[256]
        return frontDist < 0.08

    def girarIzquierda90(self):
        self.girar(math.tau/4)

    def girarDerecha90(self):
        self.girar(-math.tau/4)

    def girarMediaVuelta(self):
        self.girar(math.tau/2)

    def avanzarBaldosa(self):
        self.avanzar(0.12)

    def parar(self):
        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

    def enviarMensaje(self, pos1, pos2, letra):
        let = bytes(letra, 'utf-8')  
        mensaje = struct.pack("i i c", pos1, pos2, let) 
        self.emitter.send(mensaje)

    def enviarMensajeVoC(self, entrada):
        self.parar()
        self.delay(1200)
        self.enviarMensaje(int(self.position.x * 100), int(self.position.y * 100), entrada)

    def convertir_camara(self, img, alto, ancho):  
            img_a_convertir = np.array(np.frombuffer(img, np.uint8).reshape((alto, ancho, 4)))
            return img_a_convertir
    
    def detectar_color(r, g, b):
        colores = {
            "verde": (abs(r - 48) < 15 and abs(g - 255) < 15 and abs(b - 48) < 15),
            "rojo": (abs(r - 255) < 15 and abs(g - 91) < 15 and abs(b - 91) < 15),
            "azul": (abs(r - 91) < 15 and abs(g - 91) < 15 and abs(b - 255) < 15),
            "violeta": (abs(r - 193) < 15 and abs(g - 93) < 15 and abs(b - 251) < 15),
            "del_suelo": (abs(r - 252) < 2 and abs(g - 252) < 2 and abs(b - 252) < 2),
            "checkpoint": (abs(r - 255) < 2 and abs(g - 255) < 2 and abs(b - 255) < 2),
            "huecos": (abs(r - 60) < 15 and abs(g - 60) < 15 and abs(b - 60) < 15),
            "pantanos": (abs(r - 255) < 15 and abs(g - 222) < 15 and abs(b - 142) < 15)
        }
        for color, condicion in colores.items():
            if condicion:
                return color
        return  None

    def isVisited(self, baldosas_recorridas, posicion_inicial, pos):
        gridIndex = self.positionToGrid(posicion_inicial, pos)
        if not gridIndex in baldosas_recorridas:
            baldosas_recorridas.append(gridIndex)
            return baldosas_recorridas
        return True


    def normalizar_radianes(self, radianes): # radianes seria la rotacion actual del robot
        if radianes > math.pi:
            radianes -= math.pi*2
            return radianes
        elif radianes < -math.pi:
            radianes += math.pi*2
            return radianes
        return radianes
    
    def obtener_orientacion(self, radianes):
        angulo = self.normalizar_radianes(radianes)
        if angulo >= 0.785 and angulo <= 2.355:
            return 'W'
        elif angulo >= -2.355 and angulo <= -0.785:
            return 'E'
        elif angulo >= -0.785 and angulo <= 0.785:
            return 'N'
        else:
            return 'S'
    
    def coordenada_baldosa_derecha(self, posicion_inicial, pos, radianes):
        baldosa_actual = self.positionToGrid(posicion_inicial, pos)
        baldosa_actual = list(baldosa_actual)
        orientacion = self.obtener_orientacion(radianes)
        baldosa_derecha = []
        if orientacion == 'N':
            baldosa_derecha.append(baldosa_actual[0] + 1)
            baldosa_derecha.append(baldosa_actual[1])
        elif orientacion == 'W':
            baldosa_derecha.append(baldosa_actual[0])
            baldosa_derecha.append(baldosa_actual[1] - 1)
        elif orientacion == 'S':
            baldosa_derecha.append(baldosa_actual[0] - 1)
            baldosa_derecha.append(baldosa_actual[1])
        elif orientacion == 'E':
            baldosa_derecha.append(baldosa_actual[0])
            baldosa_derecha.append(baldosa_actual[1] + 1)
        return tuple(baldosa_derecha)
        
    def coordenada_baldosa_delantera(self, posicion_inicial, pos, radianes):
        baldosa_actual = self.positionToGrid(posicion_inicial, pos)
        list(baldosa_actual)
        orientacion = self.obtener_orientacion(radianes)
        baldosa_delantera = []
        if orientacion == 'N':
            baldosa_delantera.append(baldosa_actual[0])
            baldosa_delantera.append(baldosa_actual[1] - 1)
        elif orientacion == 'S':
            baldosa_delantera.append(baldosa_actual[0])
            baldosa_delantera.append(baldosa_actual[1] + 1)
        elif orientacion == 'W':
            baldosa_delantera.append(baldosa_actual[0] - 1)
            baldosa_delantera.append(baldosa_actual[1])
        elif orientacion == 'E':
            baldosa_delantera.append(baldosa_actual[0] + 1)
            baldosa_delantera.append(baldosa_actual[1])
        return tuple(baldosa_delantera)

    def coordenada_baldosa_izquierda(self, posicion_inicial, pos, radianes):
        baldosa_actual = self.positionToGrid(posicion_inicial, pos)
        baldosa_actual = list(baldosa_actual)
        orientacion = self.obtener_orientacion(radianes)
        baldosa_izquierda = []
        if orientacion == 'N':
            baldosa_izquierda.append(baldosa_actual[0] - 1)
            baldosa_izquierda.append(baldosa_actual[1])
        elif orientacion == 'W':
            baldosa_izquierda.append(baldosa_actual[0])
            baldosa_izquierda.append(baldosa_actual[1] + 1)
        elif orientacion == 'S':
            baldosa_izquierda.append(baldosa_actual[0] + 1)
            baldosa_izquierda.append(baldosa_actual[1])
        elif orientacion == 'E':
            baldosa_izquierda.append(baldosa_actual[0])
            baldosa_izquierda.append(baldosa_actual[1] - 1)    
        return tuple(baldosa_izquierda)
    
    def isOpenNorth(self):
        orient = self.obtener_orientacion(self.rotation)
        lidar_idx = {'N': 256,
                     'W': 384,
                     'S': 0,
                     'E': 128}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08

    def isOpenSouth(self):
        orient = self.obtener_orientacion(self.rotation)
        lidar_idx = {'S': 256,
                     'E': 384,
                     'N': 0,
                     'W': 128}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
        
    def isOpenWest(self):
        orient = self.obtener_orientacion(self.rotation)
        lidar_idx = {'S': 384,
                     'E': 0,
                     'N': 128,
                     'W': 256}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
        
    def isOpenEast(self):
        orient = self.obtener_orientacion(self.rotation)
        lidar_idx = {'S': 128,
                     'E': 256,
                     'N': 384,
                     'W': 0}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
        
    def updateMap(self):
        col, row = self.map.positionToGrid(self.position)
        tile = self.map.addTile(col, row)
        tile.visits += 1
        if self.isOpenNorth():
            north_tile = self.map.addTile(col, row - 1)
            north_tile.south = tile
            tile.north = north_tile
        if self.isOpenWest():
            west_tile = self.map.addTile(col - 1, row)
            west_tile.east = tile
            tile.west = west_tile
        if self.isOpenEast():
            east_tile = self.map.addTile(col + 1, row)
            east_tile.west = tile
            tile.east = east_tile

        if self.isOpenSouth():
            south_tile = self.map.addTile(col, row + 1)
            south_tile.north = tile
            tile.south = south_tile

    def checkNeighbours(self):
        orient = self.obtener_orientacion(self.rotation)
        col, row = self.map.positionToGrid(self.position)
        current_tile = self.map.getTileAt(col, row)

        tile_order = {"N": ((-1, 0), (0, -1), (1, 0), (0, 1)),
                      "E": ((0, -1), (1, 0), (0, 1), (-1, 0)),
                      "S": ((1, 0), (0, 1), (-1, 0), (0, -1)),
                      "W": ((0, 1), (-1, 0), (0, -1), (1, 0))}
        tiles = []
        for c, r in tile_order[orient]:
            tile = self.map.addTile(col + c, row + r)
            if tile.isConnectedTo(current_tile):
                tiles.append(tile)

        return tiles
    
    def getDirectionBetween(self, src, dst):
        sc = src.col
        sr = src.row
        dc = dst.col
        dr = dst.row
        if dc - sc == 0: # Misma columna
            if dr - sr > 0: return "S"
            if dr - sr < 0: return "N"
        elif dr - sr == 0: # Misma fila
            if dc - sc > 0: return "E"
            if dc - sc < 0: return "W"
        return None
    
    
    def moveTo(self, tile):
        col, row = self.map.positionToGrid(self.position)
        current_tile = self.map.getTileAt(col, row)

        while self.obtener_orientacion(self.rotation) != self.getDirectionBetween(current_tile, tile):
            self.girarIzquierda90()

        self.avanzarBaldosa()

    