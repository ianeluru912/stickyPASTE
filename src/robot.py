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
MAX_VEL = 6.28

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
    
        self.position = None
        self.lastPosition=None
        self.rotation = 0
        self.posicion_inicial = None

        self.step_counter = 0

        self.navigators = {1: Navigator1(), 2: Navigator2()}

        # self.holeIZ = self.imageProcessor.see_hole()
        # self.holeDER = self.imageProcessor.see_hole()
        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)
        self.map = None
        self.stepInit()
        self.posicion_inicial = self.position
        self.map = Map(self.posicion_inicial)
        self.current_area = 1
        inicio=self.map.getTileAtPosition(self.posicion_inicial)
        inicio.set_area(self.current_area)
        inicio.type = TileType.STARTING
        self.doingLOP = False

        self.mapvis = MapVisualizer()

    def getNavigator(self):
        return self.navigators[2] # TODO: Cambiar cuando anden los otros navigators 
        return self.navigators[self.current_area]

    def stepInit(self): # este step lo hace una vez al comienzo de todo
        result = self.robot.step(TIME_STEP)
        self.step_counter += 1
        self.updatePosition()
        self.updateRotation()
        self.updateLidar()
        self.updateCamerasDetection()
        return result


    def step(self):
        result = self.robot.step(TIME_STEP)
        self.step_counter += 1
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
        self.updateTiles()
        
        if self.map != None:
            x = self.position.x - self.posicion_inicial.x
            y = self.position.y - self.posicion_inicial.y
            x_valid = utils.near_multiple(x, base=0.06, tolerance=0.0025)
            y_valid = utils.near_multiple(y, base=0.06, tolerance=0.0025)
            if x_valid and y_valid:
                self.updateMap()

    def updateTiles(self):
        tile=self.getTilePointedByColorSensor()
        if not(tile is None) and tile.type is None:
            self.classifyTile(tile)
            
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
    
    def mappingVictim(self, side, token): # side = "L" o "R" token = cartelito o víctima
        #TODO: falla en el mapeo cuando detecta una víctima estando en diagonal
        orientation=self.obtener_orientacion(self.rotation)
        xRobot = self.position.x
        yRobot = self.position.y
        xRobotRel = abs(xRobot - self.posicion_inicial.x)
        yRobotRel = abs(yRobot - self.posicion_inicial.y)
        if orientation == "N" or orientation == "S":
            # si mi posición en x está en el borde entre baldosas, yo estoy viendo la víctima en una pared interna
            # si no, lo estoy viendo en una pared externda
            #if xRobotRel is near a multiple of 0.12
            if not(utils.near_multiple(xRobotRel, 0.12, 0.03)):
                # print("NS Estoy en el borde entre dos baldosas", xRobotRel, yRobotRel)
                # Estoy caminando entre dos baldosas, es decir, estoy viendo algo en una pared interna vertical
                if side == "L":
                    if orientation == "N":
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x-0.02, self.position.y))
                    else: # estoy yendo para el sur
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x+0.02, self.position.y))
                else: # estoy viendo con la cámara derecha
                    if orientation == "N":
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x+0.02, self.position.y))   
                    else: # estoy yendo para el sur
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x-0.02, self.position.y))

                # para saber si es la pared interna vertical de arriba o de abajo, voy a fijarme si estoy más arriba o más
                # abajo de la mitad de la baldosa
                yTtoTag=self.map.gridToPosition(tileToTag.col, tileToTag.row).y
                if yRobot<yTtoTag:
                    tileToTag.tokensVerticalInternalWall[0]=token
                else:
                    tileToTag.tokensVerticalInternalWall[1]=token
            else:
                # Estoy caminando en el medio de una baldosa
                # print("NS Estoy en el medio de una baldosa", xRobotRel, yRobotRel)
                tileToTag=self.map.getTileAtPosition(self.position)
                if side == "L": # es la cámara izquierda
                    if orientation == "N":                    
                        wall=tileToTag.tokensWest
                    else: # estoy yendo para el sur
                        wall=tileToTag.tokensEast
                else:
                    if orientation == "N":
                        wall=tileToTag.tokensEast
                    else: # estoy yendo para el sur
                        wall=tileToTag.tokensWest
                yTtoTag=tileToTagPosition=self.map.gridToPosition(tileToTag.col, tileToTag.row).y 
                if yRobot< yTtoTag-0.02: #Considero que estoy en la parte superior de la baldosa
                    wall[0]=token
                elif yRobot> yTtoTag+0.02: # PArte inferior
                    wall[2]=token
                else: # En el medio de la pared
                    wall[1]=token 
        else: # orientación de E a W
            # si mi posición en y está en el borde entre baldosas, yo estoy viendo la víctima en una pared interna
            # si no, lo estoy viendo en una pared externa
            if not(utils.near_multiple(yRobotRel, 0.12, 0.03)): 
                # print("EO Estoy en el borde entre dos baldosas", yRobotRel, yRobot)
                # Estoy caminando entre dos baldosas, es decir, estoy viendo algo en una pared interna horizontal
                if side == "L":
                    if orientation == "E":
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x, self.position.y-0.02))
                    else: # estoy yendo para el oeste
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x, self.position.y+0.02))
                else: # estoy viendo con la cámara derecha
                    if orientation == "E":
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x, self.position.y+0.02))   
                    else: # estoy yendo para el sur
                        tileToTag=self.map.getTileAtPosition(Point(self.position.x, self.position.y-0.02))

                # para saber si es la pared interna horizontal de izq o derecha, voy a fijarme si estoy más izq
                # o derecha de la mitad de la baldosa
                xTtoTag=self.map.gridToPosition(tileToTag.col, tileToTag.row).x
                if xRobot<xTtoTag:
                    tileToTag.tokensHorizontalInternalWall[0]=token
                else:
                    tileToTag.tokensHorizontalInternalWall[1]=token
            else:
                # Estoy caminando en el medio de una baldosa
                # print("EO Estoy en el medio de una baldosa", xRobotRel, yRobotRel)
                tileToTag=self.map.getTileAtPosition(self.position)
                if side == "L": # es la cámara izquierda
                    if orientation == "E":                    
                        wall=tileToTag.tokensNorth
                    else: # estoy yendo para el oeste
                        wall=tileToTag.tokensSouth
                else:
                    if orientation == "E":
                        wall=tileToTag.tokensSouth
                    else: # estoy yendo para el oeste
                        wall=tileToTag.tokensNorth
                xTtoTag=tileToTagPosition=self.map.gridToPosition(tileToTag.col, tileToTag.row).x 
                if xRobot< xTtoTag-0.02: #Considero que estoy en la parte inferior de la baldosa
                    wall[0]=token
                elif xRobot> xTtoTag+0.02: # PArte superior
                    wall[2]=token
                else: # En el medio de la pared
                    wall[1]=token 

    def enviar_mensaje_imgs(self):
        if self.lidar.hayAlgoIzquierda():
            entrada_I = self.imageProcessor.procesar(self.convertir_camara(self.camI.getImage(), 64, 64))
            if entrada_I is not None:
                self.mappingVictim("L", entrada_I)
                self.enviarMensajeVoC(entrada_I)
        
        if self.lidar.hayAlgoDerecha():
            entrada_D = self.imageProcessor.procesar(self.convertir_camara(self.camD.getImage(), 64, 64))
            if entrada_D is not None:
                self.mappingVictim("R", entrada_D)
                self.enviarMensajeVoC(entrada_D)
    
    def updatePosition(self):
        x, _, y = self.gps.getValues()
        
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
                        # print(f"Robot ahora en area {self.current_area} en el tile ({currentTile.col}, {currentTile.row})")
                    
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

            if distance > 0:
                ray_idx, dist = self.lidar.getNearestObstacle()
                if ray_idx is not None:
                    ray_offset = 256 - ray_idx
                    delta_angle = ray_offset * (2*math.pi/512)
                    angle = self.normalizar_radianes(self.rotation + delta_angle)
                    target_point = utils.targetPoint(self.position, angle, dist)
                    # print(f"position: {self.position}, rotation: {self.rotation}")
                    # print(f"ray_idx: {ray_idx}, dist: {dist}")
                    # print(f"ray_offset: {ray_offset}, delta_angle: {delta_angle}")
                    # print(f"angle: {angle}")
                    # print(f"target_point: {target_point}")
                    self.map.addObstacle(target_point)
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
            # print(f"{self.step_counter} -> CASO 1")
            self.updateMap1(tiles_intersecting[0])
        elif len(tiles_intersecting) == 2:
            direction = tiles_intersecting[0].getDirectionTo(tiles_intersecting[1])
            if direction == "S" or direction == "N":
                # Caso 2
                # print(f"{self.step_counter} -> CASO 2")
                self.lidar.updateWalls2(self.rotation, self.map, tiles_intersecting)
            else:
                # Caso 3
                # print(f"{self.step_counter} -> CASO 3")
                self.lidar.updateWalls3(self.rotation, self.map, tiles_intersecting)
        elif len(tiles_intersecting) >= 3:
            # print(f"{self.step_counter} -> CASO 4")
            # print('valores rayitos:', self.lidar.ver_walls(self.rotation))
            # print('---')
            # print('valores paredes:', self.lidar.get_walls_4(self.rotation))
            self.lidar.updateWalls4(self.rotation, self.map, tiles_intersecting)

        self.mapvis.send_map(self.map)

    def updateMap1(self, tile):
        self.lidar.updateWalls1(self.rotation, self.map, tile)
        # self.classifyNeighboursTile()


    def classifyTile(self, tile):
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
            elif m.estandar():
                tile.type = TileType.STANDARD

            # if tile.type is not None:
            #     print(f"Acabo de clasificar el tile ({tile.col}, {tile.row}) como {tile.type}")    


        
    def classifyNeighboursTile(self):
        tile_ahead=self.get_tile_ahead()
        self.classifyTile(tile_ahead)
        # if self.bh_izq():
        #     tile_izq = self.get_tile_izq()
        #     tile_izq.type = TileType.BLACK_HOLE
        # if self.bh_der():
        #     tile_der = self.get_tile_der()
        #     tile_der.type = TileType.BLACK_HOLE
           
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

    def getTilePointedByColorSensor(self):
    
        if(self.lidar.rangeImage[256]>0.083):
            pointCS=utils.targetPoint(self.position, self.rotation, 0.083)
            # print(f"Yo estoy en {self.position}, con rotación {self.rotation}, y voy a ver en {pointCS}")
            
            return self.map.getTileAtPosition(pointCS)
        else:
            # Tengo algo delante que no me deja ver el tile
            return None

    def moveToPoint(self, target_pos):
        target_vector = Point(target_pos.x - self.position.x, target_pos.y - self.position.y)
        target_ang = target_vector.angle()
        delta_ang = self.normalizar_radianes(target_ang - self.rotation)
        self.girar(delta_ang)
        # self.classifyNeighboursTile()
        # tileDestino=self.get_tile_ahead() # TODO: Esto me parece que debería ser el tile que esté en target_pos
        tileDestino=self.map.getTileAtPosition(target_pos)

        
        if tileDestino.hasObstacle or tileDestino.type == TileType.BLACK_HOLE:
            # print("Hay un obstaculo o agujero en el camino")
            pass
        else:
            self.avanzar(target_vector.length())

    def getRectangle(self):
        diameter = 0.07
        left = self.position.x - diameter/2
        right = self.position.x + diameter/2
        top = self.position.y - diameter/2
        bottom = self.position.y + diameter/2
        return Rectangle(top, left, bottom, right)
