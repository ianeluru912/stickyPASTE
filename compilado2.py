#!/usr/bin/env python
import contextlib as __stickytape_contextlib

@__stickytape_contextlib.contextmanager
def __stickytape_temporary_dir():
    import tempfile
    import shutil
    dir_path = tempfile.mkdtemp()
    try:
        yield dir_path
    finally:
        shutil.rmtree(dir_path)

with __stickytape_temporary_dir() as __stickytape_working_dir:
    def __stickytape_write_module(path, contents):
        import os, os.path

        def make_package(path):
            parts = path.split("/")
            partial_path = __stickytape_working_dir
            for part in parts:
                partial_path = os.path.join(partial_path, part)
                if not os.path.exists(partial_path):
                    os.mkdir(partial_path)
                    with open(os.path.join(partial_path, "__init__.py"), "wb") as f:
                        f.write(b"\n")

        make_package(os.path.dirname(path))

        full_path = os.path.join(__stickytape_working_dir, path)
        with open(full_path, "wb") as module_file:
            module_file.write(contents)

    import sys as __stickytape_sys
    __stickytape_sys.path.insert(0, __stickytape_working_dir)

    __stickytape_write_module('robot.py', b'from controller import Robot as WebotsRobot\r\nfrom image import ImageProcessor\r\nfrom point import Point\r\nimport math\r\nimport utils\r\nimport struct\r\nimport numpy as np\r\n\r\nTIME_STEP = 16\r\nMAX_VEL = 3.14  # Reduzco la velocidad para minimizar desv\xc3\xado\r\n\r\nclass Robot:\r\n    def __init__(self):\r\n\r\n        self.robot = WebotsRobot()\r\n        self.emitter = self.robot.getDevice("emitter")\r\n        self.wheelL = self.robot.getDevice("wheel1 motor")\r\n        self.wheelL.setPosition(float("inf"))\r\n\r\n        self.wheelR = self.robot.getDevice("wheel2 motor")\r\n        self.wheelR.setPosition(float("inf"))\r\n\r\n        self.lidar = self.robot.getDevice("lidar")\r\n        self.lidar.enable(TIME_STEP)\r\n\r\n        self.inertialUnit = self.robot.getDevice("inertial_unit")\r\n        self.inertialUnit.enable(TIME_STEP)\r\n\r\n        self.gps = self.robot.getDevice("gps")\r\n        self.gps.enable(TIME_STEP)\r\n\r\n        self.colorSensor = self.robot.getDevice("colour_sensor")\r\n        self.colorSensor.enable(TIME_STEP)\r\n\r\n        self.camI = self.robot.getDevice("camaraIzquierda")\r\n        self.camI.enable(TIME_STEP)\r\n\r\n        self.camD = self.robot.getDevice("camaraDerecha")\r\n        self.camD.enable(TIME_STEP)\r\n\r\n        self.imageProcessor = ImageProcessor()\r\n\r\n        self.position = None\r\n        self.rotation = 0\r\n        self.rangeImage = None\r\n\r\n        self.wheelL.setVelocity(0)\r\n        self.wheelR.setVelocity(0)\r\n        self.step()\r\n\r\n    def step(self):\r\n        result = self.robot.step(TIME_STEP)\r\n        self.updateVars()\r\n        return result\r\n\r\n    def delay(self, ms):\r\n        initTime = self.robot.getTime()\r\n        while self.step() != -1:\r\n            if (self.robot.getTime() - initTime) * 1000.0 >= ms:\r\n                break\r\n\r\n    def updateVars(self):\r\n        self.updatePosition()\r\n        self.updateRotation()\r\n        self.updateRangeImage()\r\n\r\n    def updatePosition(self):\r\n        x, _, y = self.gps.getValues()\r\n        self.position = Point(x, y)\r\n\r\n    def updateRotation(self):\r\n        _, _, yaw = self.inertialUnit.getRollPitchYaw()\r\n        self.rotation = yaw % math.tau  # Normalizamos el valor del \xc3\xa1ngulo (0 a 2*PI)\r\n\r\n    def updateRangeImage(self):\r\n        self.rangeImage = self.lidar.getRangeImage()[1024:1536]\r\n\r\n    def girar(self, rad):\r\n        lastRot = self.rotation\r\n        deltaRot = 0\r\n\r\n        while self.step() != -1:\r\n            deltaRot += utils.angle_diff(self.rotation, lastRot)\r\n            lastRot = self.rotation\r\n\r\n            diff = utils.angle_diff(deltaRot, abs(rad))\r\n\r\n            mul = (5/math.pi) * diff\r\n            mul = min(max(mul, 0.05), 1)\r\n\r\n            if rad > 0:\r\n                self.wheelL.setVelocity(mul*MAX_VEL)\r\n                self.wheelR.setVelocity(-mul*MAX_VEL)\r\n            else:\r\n                self.wheelL.setVelocity(-mul*MAX_VEL)\r\n                self.wheelR.setVelocity(mul*MAX_VEL)\r\n\r\n            if diff <= 0.005:\r\n                break\r\n\r\n        self.wheelL.setVelocity(0)\r\n        self.wheelR.setVelocity(0)\r\n\r\n    def avanzar(self, distance):\r\n        initPos = self.position\r\n\r\n        while self.step() != -1:\r\n            diff = abs(distance) - initPos.distance_to(self.position)\r\n\r\n            vel = min(max(diff/0.01, 0.1), 1)\r\n            if distance < 0: vel *= -1\r\n\r\n            self.wheelL.setVelocity(vel*MAX_VEL)\r\n            self.wheelR.setVelocity(vel*MAX_VEL)\r\n\r\n            if diff < 0.001:\r\n                break\r\n\r\n        self.wheelL.setVelocity(0)\r\n        self.wheelR.setVelocity(0)\r\n\r\n    def hayAlgoIzquierda(self):\r\n        leftDist = self.rangeImage[128]\r\n        return leftDist < 0.08\r\n\r\n    def hayAlgoDerecha(self):\r\n        rightDist = self.rangeImage[128*3]\r\n        return rightDist < 0.08\r\n\r\n    def hayAlgoAdelante(self):\r\n        frontDist = self.rangeImage[256]\r\n        return frontDist < 0.08\r\n\r\n    def girarIzquierda90(self):\r\n        self.girar(math.tau/4)\r\n\r\n    def girarDerecha90(self):\r\n        self.girar(-math.tau/4)\r\n\r\n    def girarMediaVuelta(self):\r\n        self.girar(math.tau/2)\r\n\r\n    def avanzarBaldosa(self):\r\n        self.avanzar(0.12)\r\n\r\n    def parar(self):\r\n        self.wheelL.setVelocity(0)\r\n        self.wheelR.setVelocity(0)\r\n\r\n    def enviarMensaje(self, pos1, pos2, letra):\r\n        let = bytes(letra, \'utf-8\')  \r\n        mensaje = struct.pack("i i c", pos1, pos2, let) \r\n        self.emitter.send(mensaje)\r\n\r\n    def enviarMensajeVoC(self, entrada):\r\n        self.parar()\r\n        self.delay(1200)\r\n        self.enviarMensaje(int(self.position.x * 100), int(self.position.y * 100), entrada)\r\n\r\n    def convertir_camara(self, img, alto, ancho):  \r\n            img_a_convertir = np.array(np.frombuffer(img, np.uint8).reshape((alto, ancho, 4)))\r\n            return img_a_convertir\r\n\r\n')
    __stickytape_write_module('image.py', b"import cv2\r\nimport numpy as np\r\n\r\n\r\nclass ImageProcessor:\r\n\r\n    def __init__(self):\r\n        self.letra_img = None\r\n        # self.img_a_convertir = None\r\n\r\n\r\n    def es_victima(self):\r\n        salida = None\r\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\r\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\r\n        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n        contorno_img = np.copy(self.img)\r\n        for cont in contours:\r\n            cv2.drawContours(contorno_img, [cont], -1, (0, 255, 0), 2)\r\n        return contours if len(contours) == 1 and len(contours[0]) <= 10 else salida\r\n\r\n    def devolver_letra_victimas(self):\r\n        salida = None\r\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\r\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\r\n        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n        thresh_img = np.copy(thresh)\r\n        for cont in contours:\r\n            cv2.drawContours(thresh_img, [cont], -1, (0, 255, 0), 2)\r\n        if len(contours) == 1 and len(contours[0]) <= 10:\r\n            approx = cv2.minAreaRect(contours[0])\r\n            angulo = approx[2]\r\n            if angulo % 90 == 0:\r\n                x = int(approx[0][0])\r\n                y = int(approx[0][1])\r\n                mitad_ancho = int(approx[1][0] / 2)\r\n                mitad_alto = int(approx[1][1] / 2)\r\n                cuadritoArriba = thresh[y - mitad_alto:y - int(mitad_alto / 3), x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]\r\n                cuadritoAbajo = thresh[y + int(mitad_alto / 3):y + mitad_alto, x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]\r\n                pixeles_negros_arriba = np.count_nonzero(cuadritoArriba == 0)\r\n                pixeles_negros_abajo = np.count_nonzero(cuadritoAbajo == 0)\r\n                if pixeles_negros_abajo <= 5 and pixeles_negros_arriba <= 5:\r\n                    salida = 'H'\r\n                elif pixeles_negros_abajo >= 13 and pixeles_negros_arriba >= 13:\r\n                    salida = 'S'\r\n                elif pixeles_negros_abajo >= 15 and pixeles_negros_arriba <= 5:\r\n                    salida = 'U'\r\n                return salida\r\n        return salida\r\n\r\n    def reconocer_limpiar_cartel(self):\r\n        salida = None\r\n        gris=cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\r\n        _, thresh=cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\r\n        contours,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n        approx=cv2.minAreaRect(contours[0])\r\n        angulo=approx[2]\r\n        if abs(angulo)==45:\r\n            alto, ancho=thresh.shape[0], thresh.shape[1]\r\n            M=cv2.getRotationMatrix2D((ancho/2,alto/2),angulo,1)\r\n            thresh_rot=cv2.warpAffine(thresh,M,(ancho,alto))\r\n            imagen_rot=cv2.warpAffine(self.img,M,(ancho,alto))\r\n            contours,_ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n            approx=cv2.minAreaRect(contours[0])\r\n            x=int(approx[0][0])\r\n            y=int(approx[0][1])\r\n            mitad_ancho=int(approx[1][0]/2)\r\n            mitad_alto=int(approx[1][1]/2)\r\n            rect=imagen_rot[y-mitad_alto:y+mitad_alto, x-mitad_ancho:x+mitad_ancho]\r\n            return rect, True\r\n        return salida   \r\n\r\n    def devolver_letra_carteles(self):\r\n        salida = None\r\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\r\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\r\n        contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n        if len(contornos) == 0:\r\n            return None\r\n        approx = cv2.minAreaRect(contornos[0])\r\n        angulo = approx[2]\r\n        if abs(angulo) == 45:\r\n            alto, ancho = thresh.shape[0], thresh.shape[1]\r\n            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)\r\n            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))\r\n            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))\r\n            contornos, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\r\n            if len(contornos) == 0:\r\n                return None\r\n            approx = cv2.minAreaRect(contornos[0])\r\n            x = int(approx[0][0])\r\n            y = int(approx[0][1])\r\n            mitadAncho = int(approx[1][0] / 2)\r\n            mitadAlto = int(approx[1][1] / 2)\r\n            if y - mitadAlto < 0 or y + mitadAlto > imagen_rot.shape[0] or x - mitadAncho < 0 or x + mitadAncho > imagen_rot.shape[1]:\r\n                return None\r\n            rect = imagen_rot[y - mitadAlto:y + mitadAlto, x - mitadAncho:x + mitadAncho]\r\n            amarillo, rojo, negro, blanco = 0, 0, 0, 0\r\n            for x in range(rect.shape[0]):\r\n                for y in range(rect.shape[1]):\r\n                    b, g, r = rect[x, y]\r\n                    if b > 200 and g > 200 and r > 200:\r\n                        blanco += 1\r\n                    elif b <= 1 and g <= 1 and r <= 1:\r\n                        negro += 1\r\n                    elif b > 70 and g < 5 and r > 190:\r\n                        rojo += 1\r\n                    elif b < 10 and g > 190 and r > 195:\r\n                        amarillo += 1\r\n            if (rojo + blanco) > (negro + amarillo) and rojo > blanco and blanco > negro:\r\n                return salida == 'F'\r\n            elif (blanco + negro) > (amarillo + rojo) and blanco > negro:\r\n                return salida == 'P'\r\n            elif (blanco + negro) > (amarillo + rojo):\r\n                return salida == 'C'\r\n            elif (rojo + amarillo) > (negro + blanco):\r\n                return salida == 'O'\r\n            return salida\r\n        return salida\r\n\r\n# Nuevo argumento\r\n    def procesar(self, resultado):\r\n        self.letra = None\r\n        self.img = resultado\r\n        if self.es_victima() is not None:\r\n            return self.letra_img == self.devolver_letra_victimas()\r\n        else:\r\n            resultado = self.reconocer_limpiar_cartel()\r\n            if resultado is not None:\r\n                return self.letra_img == self.devolver_letra_carteles()\r\n            return self.letra_img\r\n\r\n\r\n\r\n")
    __stickytape_write_module('point.py', b'import math\r\n\r\nclass Point:\r\n    def __init__(self, x, y):\r\n        self.x = x\r\n        self.y = y\r\n\r\n    def distance_to(self, point):\r\n        dx = self.x-point.x\r\n        dy = self.y-point.y\r\n        return math.sqrt(dx**2 + dy**2)\r\n    \r\n    def __str__(self) -> str:\r\n        return f"({self.x:.3f}, {self.y:.3f})"')
    __stickytape_write_module('utils.py', b'import math\r\n\r\ndef angle_diff(a, b):\r\n    clockwise = (a - b) % math.tau\r\n    counterclockwise = (b - a) % math.tau\r\n    return min(clockwise, counterclockwise)')
    from robot import Robot
    from image import ImageProcessor
    
    # Inicializar el robot y procesar la imagen obtenida
    robot = Robot()
    image_processor = ImageProcessor()  
    
    # Obtener lo leido de la camara -
    
    # entrada = image_processor.procesar(robot.img_a_convertir, image_processor.letra_img)
    
    while robot.step() != -1:
        img = robot.camI.getImage() 
        alto = robot.camI.getHeight()
        ancho = robot.camI.getWidth()
        resultado = robot.convertir_camara(img, alto, ancho)
        entrada = image_processor.procesar(resultado) #River perdi� con argentinos juniors
        if not robot.hayAlgoIzquierda():
            robot.girarIzquierda90() 
            image_processor.procesar(resultado)
            if entrada is not None:
                robot.enviarMensajeVoC(entrada)
            robot.avanzarBaldosa()
            image_processor.procesar(resultado)
            if entrada is not None:
                robot.enviarMensajeVoC(entrada)
        elif not robot.hayAlgoAdelante():
            robot.avanzarBaldosa()
            image_processor.procesar(resultado)
            if entrada is not None: 
                robot.enviarMensajeVoC(entrada)
        elif not robot.hayAlgoDerecha():
            robot.girarDerecha90()
            image_processor.procesar(resultado)
            if entrada is not None:
                robot.enviarMensajeVoC(entrada)
            robot.avanzarBaldosa()
            image_processor.procesar(resultado)
            if entrada is not None:
                robot.enviarMensajeVoC(entrada)
        else:
            robot.girarMediaVuelta()
            robot.avanzarBaldosa()
            image_processor.procesar(resultado)
            if entrada is not None:
                robot.enviarMensajeVoC(entrada)
    