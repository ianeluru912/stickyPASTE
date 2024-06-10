from controller import Robot as WebotsRobot
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

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)
        self.step()

        self.posicion_inicial = None

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
        self.rotation = yaw % math.tau  # Normalizamos el valor del ángulo (0 a 2*PI)

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
        return leftDist < 0.08

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128*3]
        return rightDist < 0.08

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
    def evaluar_baldosa_delantera(self):
        coordenada_y = self.position.y
        baldosa_delantera_a_evaluar = coordenada_y - 0.12
        return baldosa_delantera_a_evaluar

    def evaluar_baldosa_derecha(self):
        coordenada_x = self.position.x
        baldosa_derecha_a_evaluar = coordenada_x + 0.12
        return baldosa_derecha_a_evaluar

    def evaluar_baldosa_izquierda(self):
        coordenada_x = self.position.x
        baldosa_izquierda_a_evaluar = coordenada_x - 0.12
        return baldosa_izquierda_a_evaluar

    def isVisited(self, baldosas_recorridas, posicion_inicial):
        gridIndex = self.positionToGrid(posicion_inicial)
        if not gridIndex in baldosas_recorridas:
            baldosas_recorridas.append(gridIndex)
            return baldosas_recorridas
        return True

    def positionToGrid(self, posicion_inicial):
        grilla = []
        columna = round((self.position.x - posicion_inicial['x']) / 0.12)
        grilla.append(columna)
        fila = round((self.position.y - posicion_inicial['y']) / 0.12) 
        grilla.append(fila)
        tupla_grilla = tuple(grilla)
        return tupla_grilla

    def coordenada_baldosa_delantera(self, posicion_inicial):
        baldosa_actual = self.positionToGrid(posicion_inicial)
        list(baldosa_actual)
        baldosa_delantera = []
        baldosa_delantera.append(baldosa_actual[0])
        baldosa_delantera.append(baldosa_actual[1] - 1)
        return baldosa_delantera
    
    def coordenada_baldosa_izquierda(self, posicion_inicial):
        baldosa_actual = self.positionToGrid(posicion_inicial)
        list(baldosa_actual)
        baldosa_izquierda = []
        baldosa_izquierda.append(baldosa_actual[0] + 1)
        baldosa_izquierda.append(baldosa_actual[1])
        return baldosa_izquierda
    
    def coordendada_baldosa_derecha(self, posicion_inicial):
        baldosa_actual = self.positionToGrid(posicion_inicial)
        list(baldosa_actual)
        baldosa_derecha = []
        baldosa_derecha.append(baldosa_actual[0] - 1)
        baldosa_derecha.append(baldosa_actual[1])
        return baldosa_derecha
