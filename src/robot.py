from controller import Robot as WebotsRobot #type: ignore
from map import Map, Tile
from image import ImageProcessor
from point import Point
from piso import Piso
from lidar import Lidar
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

        self.lidar = Lidar(self.robot.getDevice("lidar"), TIME_STEP)

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
    
        self.point = Point(0, 0)

        self.position = None
        self.rotation = 0
        self.rangeImage = None
        self.posicion_inicial = None

        # self.holeIZ = self.imageProcessor.see_hole()
        # self.holeDER = self.imageProcessor.see_hole()
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
        while self.robot.step(TIME_STEP) != -1:
            if (self.robot.getTime() - initTime) * 1000.0 >= ms:
                break

    def updateVars(self):
        self.updatePosition()
        self.updateRotation()
        self.updateLidar()
        self.updateCamerasDetection()

    def updateCamerasDetection(self):
        return self.enviar_mensaje_imgs()

    def enviarMensaje(self, pos1, pos2, letra):
        let = bytes(letra, 'utf-8')  
        mensaje = struct.pack("i i c", pos1, pos2, let) 
        self.emitter.send(mensaje)

    def enviarMensajeVoC(self, entrada):
        self.parar()
        self.delay(1500)
        self.enviarMensaje(int(self.position.x * 100), int(self.position.y * 100), entrada)
        self.delay(100)

    def convertir_camara(self, img, alto, ancho):  
            img_a_convertir = np.array(np.frombuffer(img, np.uint8).reshape((alto, ancho, 4)))
            return img_a_convertir
    
    def enviar_mensaje_imgs(self):
        if self.lidar.hayAlgoIzquierda():
            entrada_I = self.imageProcessor.procesar(self.convertir_camara(self.camI.getImage(), 64, 64))
            if entrada_I is not None:
                self.enviarMensajeVoC(entrada_I)
        
        if self.lidar.hayAlgoDerecha():
            entrada_D = self.imageProcessor.procesar(self.convertir_camara(self.camD.getImage(), 64, 64))
            if entrada_D is not None:
                self.enviarMensajeVoC(entrada_D)

    def updatePosition(self):
        x, _, y = self.gps.getValues()
        self.position = Point(x, y)

    def updateRotation(self):
        _, _, yaw = self.inertialUnit.getRollPitchYaw()
        self.rotation = self.normalizar_radianes(yaw % math.tau)

    def updateLidar(self):
        self.lidar.update()

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

    def bh_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.blackHole():
            return True
        else:
            return False
    
    def pantano_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.pantano():
            return True
        else:
            return False
    def suelo_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.del_suelo():
            return True
        else:
            return False
    def azul_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.azul():
            return True
        else:
            return False
    def verde_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.verde():
            return True
        else:
            return False
    def violeta_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.violeta():
            return True
        else:
            return False
    def rojo_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.rojo():
            return True
        else:
            return False
    def checkpoint_ahead(self):
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if m.checkpoint():
            return True
        else:
            return False
    
    def bh_izq(self):
        orientation = self.obtener_orientacion(self.rotation)
        if orientation == 'N':
            if self.isOpenWest():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camI.getImage(), 64, 64))
            return False
        elif orientation == 'E':
            if self.isOpenNorth():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camI.getImage(), 64, 64))
            return False
        elif orientation == 'S':
            if self.isOpenEast():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camI.getImage(), 64, 64))
            return False
        elif orientation == 'W':
            if self.isOpenSouth():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camI.getImage(), 64, 64))
            return False   

    def bh_der(self):
        orientation = self.obtener_orientacion(self.rotation)
        if orientation == 'N':
            if self.isOpenEast():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camD.getImage(), 64, 64))
            return False
        elif orientation == 'E':
            if self.isOpenSouth():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camD.getImage(), 64, 64))
            return False
        elif orientation == 'S':
            if self.isOpenWest():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camD.getImage(), 64, 64))
            return False
        elif orientation == 'W':
            if self.isOpenNorth():
                return self.imageProcessor.see_hole(self.convertir_camara(self.camD.getImage(), 64, 64))
            return False

    def parar(self):
        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

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
        
    def isOpenNorth(self):
        orient = self.obtener_orientacion(self.rotation)
        return self.lidar.isOpenNorth(orient)        

    def isOpenSouth(self):
        orient = self.obtener_orientacion(self.rotation)
        return self.lidar.isOpenSouth(orient)
        
    def isOpenWest(self):
        orient = self.obtener_orientacion(self.rotation)
        return self.lidar.isOpenWest(orient)
        
    def isOpenEast(self):
        orient = self.obtener_orientacion(self.rotation)
        return self.lidar.isOpenEast(orient)
    
    def get_tile_ahead(self):
        col, row = self.map.positionToGrid(self.position)
        orient = self.obtener_orientacion(self.rotation)
        if orient == "N":
            return self.map.getTileAt(col, row - 1)
        elif orient == "S":
            return self.map.getTileAt(col, row + 1)
        elif orient == "E":            
            return self.map.getTileAt(col + 1, row)
        elif orient == "W":            
            return self.map.getTileAt(col - 1, row)
        
    def get_tile_izq(self):
        col, row = self.map.positionToGrid(self.position)
        orient = self.obtener_orientacion(self.rotation)
        if orient == "N":
            return self.map.getTileAt(col - 1, row)
        elif orient == "S":
            return self.map.getTileAt(col + 1, row)
        elif orient == "E":            
            return self.map.getTileAt(col, row - 1)
        elif orient == "W":
            return self.map.getTileAt(col, row + 1)
    def get_tile_der(self):
        col, row = self.map.positionToGrid(self.position)
        orient = self.obtener_orientacion(self.rotation)
        if orient == "N":
            return self.map.getTileAt(col + 1, row)
        elif orient == "S":
            return self.map.getTileAt(col - 1, row)
        elif orient == "E":
            return self.map.getTileAt(col, row + 1)
        elif orient == "W":
            return self.map.getTileAt(col, row - 1)

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
        if self.bh_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isBlackHole = True
        if self.bh_izq():
            tile_izq = self.get_tile_izq()
            tile_izq.isBlackHole = True
        if self.bh_der():
            tile_der = self.get_tile_der()
            tile_der.isBlackHole = True
        if self.pantano_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isSwamp = True
        if self.azul_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isBlue = True
        if self.verde_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isGreen = True
        if self.violeta_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isPurple = True
        if self.rojo_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isRed = True
        if self.checkpoint_ahead():
            tile_ahead = self.get_tile_ahead()
            tile_ahead.isCheckpoint= True
            
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
            if tile.isConnectedTo(current_tile) and not tile.isBlackHole:
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

    def moveToTile(self, tile):
        target_pos = self.map.gridToPosition(tile.col, tile.row)
        self.moveToPoint(target_pos)

    def moveToPoint(self, target_pos):
        target_vector = Point(target_pos.x - self.position.x, target_pos.y - self.position.y)
        target_ang = target_vector.angle()
        delta_ang = self.normalizar_radianes(target_ang - self.rotation)
        self.girar(delta_ang)
        self.avanzar(target_vector.length())
