from controller import Robot as WebotsRobot # type: ignore
from map import Map, Tile, TileType
from image import ImageProcessor
from point import Point
from piso import Piso
from lidar import Lidar
from rectangle import Rectangle
from navigator import Navigator1, Navigator2
import math
import utils
import struct
import numpy as np

from visualization import MapVisualizer

TIME_STEP = 16
MAX_VEL = 3.14  # Reduzco la velocidad para minimizar desvío

class Robot:
    def __init__(self):
        self.piso = Piso(0, 0, 0)
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
        self.lastPosition=None
        self.rotation = 0
        self.rangeImage = None
        self.posicion_inicial = None

        self.navigators = {1: Navigator1(), 2: Navigator2()}

        # self.holeIZ = self.imageProcessor.see_hole()
        # self.holeDER = self.imageProcessor.see_hole()
        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)
        self.map = None
        self.step()

        self.posicion_inicial = self.position
        self.map = Map(self.posicion_inicial)
        self.current_area = 1
        self.doingLOP = False

        self.mapvis = MapVisualizer()

    def getNavigator(self):
        return self.navigators[1] # TODO: Cambiar cuando anden los otros navigators 
        return self.navigators[self.current_area]

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
        
        if self.map != None:
            x = self.position.x - self.posicion_inicial.x
            y = self.position.y - self.posicion_inicial.y
            x_valid = round(x * 100) % 6 <= 0
            y_valid = round(y * 100) % 6 <= 0
            if x_valid and y_valid:
                self.updateMap()

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
        # print(self.lastPosition, self.position)
        
        self.position = Point(x, y)
        if self.lastPosition is None:
            self.lastPosition = self.position
        else:
            previousTile = self.map.getTileAtPosition(self.lastPosition)
            currentTile = self.map.getTileAtPosition(self.position)
            if currentTile != previousTile: # Entramos a una nueva baldosa
                previousTile.visits += 1

                if not self.doingLOP:
                    if currentTile.get_area() is None:
                        currentTile.set_area(self.current_area)
                        # print(f"Tile en ({tile.col}, {tile.row}) marcada en area {tile.area}")
                    else:
                        self.current_area = currentTile.get_area()
                        print(f"Robot ahora en area {self.current_area} en el tile ({currentTile.col}, {currentTile.row})")
                    
                    color = currentTile.type
                    if color:
                        self.update_area_by_color(color)
                        currentTile.set_area(self.current_area)

            if self.lastPosition.distance_to(self.position) > 0.06:
                # print("LOP")
                self.doingLOP = True
                self.current_area = currentTile.get_area()
                self.lastPosition = self.position
            else:
                self.lastPosition = self.position
                self.doingLOP = False

    def updateRotation(self):
        _, _, yaw = self.inertialUnit.getRollPitchYaw()
        self.rotation = self.normalizar_radianes(yaw % math.tau)

    def updateLidar(self):
        self.lidar.update()

    def girar(self, rad):
        lastRot = self.rotation
        deltaRot = 0

        while self.step() != -1:
            if self.doingLOP:
                self.wheelL.setVelocity(0)
                self.wheelR.setVelocity(0)
                break
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
        hasObstacle = False

        while self.step() != -1:
            if self.doingLOP:
                self.wheelL.setVelocity(0)
                self.wheelR.setVelocity(0)
                break
            diff = abs(distance) - initPos.distance_to(self.position)
            vel = min(max(diff/0.01, 0.1), 1)
            
            if distance < 0: vel *= -1

            self.wheelL.setVelocity(vel*MAX_VEL)
            self.wheelR.setVelocity(vel*MAX_VEL)

            if distance > 0 and self.lidar.is_obstacle_preventing_passage():
                # print('hay algo delante')
                hasObstacle = True
                break

            if diff < 0.001:
                break

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

        if hasObstacle:
            # 1) Obtener el tile en el que está el obstáculo
            col, row = self.map.positionToGrid(initPos)
            # TODO(Richo): Este código asume navegación de centro de baldosa a centro de baldosa.
            # Acá habría que calcular cuál es la posición destino para después pedirle al mapa qué
            # tile corresponde con esa punto.
            orient = self.obtener_orientacion(self.rotation)
            if orient == "N":
                row -= 1
            elif orient == "S":
                row += 1
            elif orient == "E":
                col += 1
            elif orient == "W":
                col -= 1
            tile = self.map.getTileAt(col, row)

            # 2) Marcar ese tile como hasObstacle = True
            tile.hasObstacle = True

            # 3) Retroceder la misma distancia que avancé
            dist = initPos.distance_to(self.position)
            self.avanzar(-dist)
    
    def bh_izq(self):
        # TODO(Richo): Este código asume navegación de centro de baldosa a centro de baldosa
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
        # TODO(Richo): Este código asume navegación de centro de baldosa a centro de baldosa
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
        # TODO(Richo): Este código asume que no nos movemos en diagonal
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
        rect = self.getRectangle()
        tiles_intersecting = self.map.getTilesIntersecting(rect)
        if len(tiles_intersecting) == 1:
            # print("CASO 1")
            self.updateMap1(tiles_intersecting[0])
        elif len(tiles_intersecting) == 2:
            direction = tiles_intersecting[0].getDirectionTo(tiles_intersecting[1])
            if direction == "S" or direction == "N":
                # Caso 2
                # print("CASO 2")
                self.lidar.updateWalls2(self.rotation, self.map, tiles_intersecting)
            else:
                # Caso 3
                # print("CASO 3")
                self.lidar.updateWalls3(self.rotation, self.map, tiles_intersecting)
        elif len(tiles_intersecting) == 4:
            # print("CASO 4")
            self.lidar.updateWalls4(self.rotation, self.map, tiles_intersecting)

        self.mapvis.send_map(self.map)

    def updateMap1(self, tile):
        self.lidar.updateWalls1(self.rotation, self.map, tile)
        self.classifyNeighbourTile()

    def classifyAhead(self, tile):
        # print(tile.type)
        b, g, r, _ = self.colorSensor.getImage()
        m = Piso(r, g, b)
        if tile.type is None:
            if m.blackHole():
                tile.type = TileType.BLACK_HOLE         
            elif m.pantano():
                tile.type = TileType.SWAMP
            elif m.blue():
                tile.type = TileType.BLUE   
            elif m.green():
                tile.type = TileType.GREEN
            elif m.purple():
                tile.type = TileType.PURPLE
            elif m.red():
                tile.type = TileType.RED
            elif m.orange():
                tile.type = TileType.ORANGE
            elif m.yellow():
                tile.type = TileType.YELLOW
            elif m.checkpoint():
                tile.type = TileType.CHECKPOINT

        
    def classifyNeighbourTile(self):
        tile_ahead=self.get_tile_ahead()
        self.classifyAhead(tile_ahead)
        if self.bh_izq():
            tile_izq = self.get_tile_izq()
            tile_izq.type = TileType.BLACK_HOLE
        if self.bh_der():
            tile_der = self.get_tile_der()
            tile_der.type = TileType.BLACK_HOLE
           
    def update_area_by_color(self, color):
        possibleAreas = {
            (1, TileType.BLUE): 2, #area 1, azul? area 2
            (1, TileType.YELLOW): 3,
            (1, TileType.GREEN): 4,
            (2, TileType.BLUE): 1, #area 2, azul? area 1
            (2, TileType.PURPLE): 3, #""
            (2, TileType.ORANGE): 4,
            (3, TileType.YELLOW): 1,
            (3, TileType.PURPLE): 2,
            (3, TileType.RED): 4,
            (4, TileType.GREEN): 1,
            (4, TileType.ORANGE): 2,
            (4, TileType.RED): 3
        }

        changeOfArea = (self.current_area, color)
        if changeOfArea in possibleAreas:
            self.current_area = possibleAreas[changeOfArea]
            # print(f"Area actualizada a {self.current_area} por color {color}")

    def moveToPoint(self, target_pos):
        target_vector = Point(target_pos.x - self.position.x, target_pos.y - self.position.y)
        target_ang = target_vector.angle()
        delta_ang = self.normalizar_radianes(target_ang - self.rotation)
        self.girar(delta_ang)
        self.classifyNeighbourTile()
        tileDestino=self.get_tile_ahead() # TODO: Esto me parece que debería ser el tile que esté en target_pos
        if tileDestino.hasObstacle or tileDestino.type == TileType.BLACK_HOLE:
            print("Hay un obstaculo o agujero en el camino")
        else:
            self.avanzar(target_vector.length())

    def getRectangle(self):
        diameter = 0.07
        left = self.position.x - diameter/2
        right = self.position.x + diameter/2
        top = self.position.y - diameter/2
        bottom = self.position.y + diameter/2
        return Rectangle(top, left, bottom, right)
