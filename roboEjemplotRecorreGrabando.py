from controller import Robot as WebotsRobot
import numpy as np
import math
import cv2

TIME_STEP = 16
MAX_VEL = 3.14 

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other_point):
        return math.sqrt((self.x - other_point.x) ** 2 + (self.y - other_point.y) ** 2)

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

        self.position = None
        self.rotation = 0
        self.rangeImage = None

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)
        
        self.nroImagen = 0
        self.pasoGrabacion = 0
        self.CADACUANTOGRABO = 3
        
        self.step()

    def convertirCamara(self, imagen, alto, ancho):
        return np.array(np.frombuffer(imagen, np.uint8).reshape((alto, ancho, 4)))

    def grabar(self):
        self.pasoGrabacion += 1
        if self.pasoGrabacion == self.CADACUANTOGRABO:
            self.pasoGrabacion = 0
            self.nroImagen += 1
            cv2.imwrite(f"CI{str(self.nroImagen).rjust(4, '0')}.png", self.convertirCamara(self.camI.getImage(), 64, 64))
            cv2.imwrite(f"CD{str(self.nroImagen).rjust(4, '0')}.png", self.convertirCamara(self.camD.getImage(), 64, 64))
            # cv2.imwrite(f"CF{str(nroImagen).rjust(4,'0')}.png",convertirCamara(camF.getImage(),128,128))
    
    def angle_diff(self, a, b):
        diff = a - b
        while diff < -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
        return diff

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
        self.rotation = yaw % math.tau  

    def updateRangeImage(self):
        self.rangeImage = self.lidar.getRangeImage()[1024:1536]

    def girar(self, rad):
        lastRot = self.rotation
        deltaRot = 0

        while self.step() != -1:
            deltaRot += self.angle_diff(self.rotation, lastRot)
            lastRot = self.rotation

            diff = self.angle_diff(deltaRot, abs(rad))

            mul = (5 / math.pi) * diff
            mul = min(max(mul, 0.05), 1)

            if rad > 0:
                self.wheelL.setVelocity(mul * MAX_VEL)
                self.wheelR.setVelocity(-mul * MAX_VEL)
            else:
                self.wheelL.setVelocity(-mul * MAX_VEL)
                self.wheelR.setVelocity(mul * MAX_VEL)

            if diff <= 0.005:
                break

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

    def avanzar(self, distance):
        initPos = self.position

        while self.step() != -1:
            diff = abs(distance) - initPos.distance_to(self.position)

            vel = min(max(diff / 0.01, 0.1), 1)
            if distance < 0:
                vel *= -1

            self.wheelL.setVelocity(vel * MAX_VEL)
            self.wheelR.setVelocity(vel * MAX_VEL)

            if diff < 0.001:
                break

        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)

    def hayAlgoIzquierda(self):
        leftDist = self.rangeImage[128]
        return leftDist < 0.08

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128 * 3]
        return rightDist < 0.08

    def hayAlgoAdelante(self):
        frontDist = self.rangeImage[256]
        return frontDist < 0.08

    def girarIzquierda90(self):
        print("Girando a la izquierda 90 grados")
        self.girar(math.pi / 2)

    def girarDerecha90(self):
        print("Girando a la derecha 90 grados")
        self.girar(-math.pi / 2)

    def girarMediaVuelta(self):
        print("Girando media vuelta")
        self.girar(math.pi)

    def avanzarBaldosa(self):
        print("Avanzando una baldosa")
        self.avanzar(0.12)

    def parar(self):
        self.wheelL.setVelocity(0)
        self.wheelR.setVelocity(0)


robot = Robot()


while robot.step() != -1:
    print(f"Adelante: {robot.hayAlgoAdelante()}, Izquierda: {robot.hayAlgoIzquierda()}, Derecha: {robot.hayAlgoDerecha()}")
    if not robot.hayAlgoIzquierda():
        robot.girarIzquierda90() 
        robot.avanzarBaldosa()
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
    elif not robot.hayAlgoDerecha():
        robot.girarDerecha90()
        robot.avanzarBaldosa()
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()




