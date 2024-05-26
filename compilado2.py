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

    __stickytape_write_module('robot.py', b'from controller import Robot as WebotsRobot\nfrom image import ImageProcessor\nfrom point import Point\nimport math\nimport utils\nimport struct\nimport numpy as np\n\nTIME_STEP = 16\nMAX_VEL = 3.14  # Reduzco la velocidad para minimizar desv\xc3\xado\n\nclass Robot:\n    def __init__(self):\n        self.converted_img = None\n\n        self.robot = WebotsRobot()\n        self.emitter = self.robot.getDevice("emitter")\n        self.wheelL = self.robot.getDevice("wheel1 motor")\n        self.wheelL.setPosition(float("inf"))\n\n        self.wheelR = self.robot.getDevice("wheel2 motor")\n        self.wheelR.setPosition(float("inf"))\n\n        self.lidar = self.robot.getDevice("lidar")\n        self.lidar.enable(TIME_STEP)\n\n        self.inertialUnit = self.robot.getDevice("inertial_unit")\n        self.inertialUnit.enable(TIME_STEP)\n\n        self.gps = self.robot.getDevice("gps")\n        self.gps.enable(TIME_STEP)\n\n        self.colorSensor = self.robot.getDevice("colour_sensor")\n        self.colorSensor.enable(TIME_STEP)\n\n        self.camI = self.robot.getDevice("camaraIzquierda")\n        self.camI.enable(TIME_STEP)\n\n        self.camD = self.robot.getDevice("camaraDerecha")\n        self.camD.enable(TIME_STEP)\n\n        self.imageProcessor = ImageProcessor()\n\n        self.position = None\n        self.rotation = 0\n        self.rangeImage = None\n\n        self.wheelL.setVelocity(0)\n        self.wheelR.setVelocity(0)\n        self.step()\n\n    def step(self):\n        result = self.robot.step(TIME_STEP)\n        self.updateVars()\n        return result\n\n    def delay(self, ms):\n        initTime = self.robot.getTime()\n        while self.step() != -1:\n            if (self.robot.getTime() - initTime) * 1000.0 >= ms:\n                break\n\n    def updateVars(self):\n        self.updatePosition()\n        self.updateRotation()\n        self.updateRangeImage()\n\n    def updatePosition(self):\n        x, _, y = self.gps.getValues()\n        self.position = Point(x, y)\n\n    def updateRotation(self):\n        _, _, yaw = self.inertialUnit.getRollPitchYaw()\n        self.rotation = yaw % math.tau  # Normalizamos el valor del \xc3\xa1ngulo (0 a 2*PI)\n\n    def updateRangeImage(self):\n        self.rangeImage = self.lidar.getRangeImage()[1024:1536]\n\n    def girar(self, rad):\n        lastRot = self.rotation\n        deltaRot = 0\n\n        while self.step() != -1:\n            deltaRot += utils.angle_diff(self.rotation, lastRot)\n            lastRot = self.rotation\n\n            diff = utils.angle_diff(deltaRot, abs(rad))\n\n            mul = (5/math.pi) * diff\n            mul = min(max(mul, 0.05), 1)\n\n            if rad > 0:\n                self.wheelL.setVelocity(mul*MAX_VEL)\n                self.wheelR.setVelocity(-mul*MAX_VEL)\n            else:\n                self.wheelL.setVelocity(-mul*MAX_VEL)\n                self.wheelR.setVelocity(mul*MAX_VEL)\n\n            if diff <= 0.005:\n                break\n\n        self.wheelL.setVelocity(0)\n        self.wheelR.setVelocity(0)\n\n    def avanzar(self, distance):\n        initPos = self.position\n\n        while self.step() != -1:\n            diff = abs(distance) - initPos.distance_to(self.position)\n\n            vel = min(max(diff/0.01, 0.1), 1)\n            if distance < 0: vel *= -1\n\n            self.wheelL.setVelocity(vel*MAX_VEL)\n            self.wheelR.setVelocity(vel*MAX_VEL)\n\n            if diff < 0.001:\n                break\n\n        self.wheelL.setVelocity(0)\n        self.wheelR.setVelocity(0)\n\n    def hayAlgoIzquierda(self):\n        leftDist = self.rangeImage[128]\n        return leftDist < 0.08\n\n    def hayAlgoDerecha(self):\n        rightDist = self.rangeImage[128*3]\n        return rightDist < 0.08\n\n    def hayAlgoAdelante(self):\n        frontDist = self.rangeImage[256]\n        return frontDist < 0.08\n\n    def girarIzquierda90(self):\n        self.girar(math.tau/4)\n\n    def girarDerecha90(self):\n        self.girar(-math.tau/4)\n\n    def girarMediaVuelta(self):\n        self.girar(math.tau/2)\n\n    def avanzarBaldosa(self):\n        self.avanzar(0.12)\n\n    def parar(self):\n        self.wheelL.setVelocity(0)\n        self.wheelR.setVelocity(0)\n\n    def enviarMensaje(self, pos1, pos2, letra):\n        let = bytes(letra, \'utf-8\')  # Convertimos la letra a bytes para poder enviarla en la estructura\n        mensaje = struct.pack("i i c", pos1, pos2, let)  # Estructura el mensaje en el formato correcto\n        self.emitter.send(mensaje)\n\n    def enviarMensajeVoC(self, tipo):\n        self.parar()\n        self.delay(1200)  # Debemos hacer una pausa antes de enviar el mensaje\n        self.enviarMensaje(int(self.position.x * 100), int(self.position.y * 100), tipo)\n\n    def convertir_camara(self, img, alto, ancho):  # Convierte la imagen de la camara a una imagen de opencv\n        if img is not None:\n            self.converted_img = np.array(np.frombuffer(img, np.uint8).reshape((alto, ancho, 4)))\n        else:\n            pass\n')
    __stickytape_write_module('image.py', b"import cv2\nimport numpy as np\n\nclass ImageProcessor:\n    def __init__():\n        salida = None\n\n    def es_victima(self):\n        if self.img is None:\n            return None\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\n        _, thresh = cv2.threshold(gris, 127, 255, cv2.THRESH_BINARY)\n        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        contorno_img = np.copy(self.img)\n        for cont in contours:\n            cv2.drawContours(contorno_img, [cont], -1, (0, 255, 0), 2)\n        return contours if len(contours) == 1 and len(contours[0]) <= 10 else None\n\n    def devolver_letra_victimas(self):\n        salida = None\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\n        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        thresh_img = np.copy(thresh)\n        for cont in contours:\n            cv2.drawContours(thresh_img, [cont], -1, (0, 255, 0), 2)\n        if len(contours) == 1 and len(contours[0]) <= 10:\n            approx = cv2.minAreaRect(contours[0])\n            angulo = approx[2]\n            if angulo % 90 == 0:\n                x = int(approx[0][0])\n                y = int(approx[0][1])\n                mitad_ancho = int(approx[1][0] / 2)\n                mitad_alto = int(approx[1][1] / 2)\n                cuadritoArriba = thresh[y - mitad_alto:y - int(mitad_alto / 3), x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]\n                cuadritoAbajo = thresh[y + int(mitad_alto / 3):y + mitad_alto, x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]\n                pixeles_negros_arriba = np.count_nonzero(cuadritoArriba == 0)\n                pixeles_negros_abajo = np.count_nonzero(cuadritoAbajo == 0)\n                if pixeles_negros_abajo <= 5 and pixeles_negros_arriba <= 5:\n                    salida = 'H'\n                elif pixeles_negros_abajo >= 13 and pixeles_negros_arriba >= 13:\n                    salida = 'S'\n                elif pixeles_negros_abajo >= 15 and pixeles_negros_arriba <= 5:\n                    salida = 'U'\n                return salida\n        return salida\n\n    def reconocer_limpiar_cartel(self):\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\n        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        if len(contours) == 0:\n            return None\n        approx = cv2.minAreaRect(contours[0])\n        angulo = approx[2]\n        if abs(angulo) == 45:\n            alto, ancho = thresh.shape[0], thresh.shape[1]\n            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)\n            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))\n            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))\n            contours, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n            if len(contours) == 0:\n                return None\n            approx = cv2.minAreaRect(contours[0])\n            x = int(approx[0][0])\n            y = int(approx[0][1])\n            mitad_ancho = int(approx[1][0] / 2)\n            mitad_alto = int(approx[1][1] / 2)\n            if y - mitad_alto < 0 or y + mitad_alto > imagen_rot.shape[0] or x - mitad_ancho < 0 or x + mitad_ancho > imagen_rot.shape[1]:\n                return None\n            rect = imagen_rot[y - mitad_alto:y + mitad_alto, x - mitad_ancho:x + mitad_ancho]\n            return rect, True\n        return None\n\n    def devolver_letra_carteles(self):\n        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)\n        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)\n        contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        if len(contornos) == 0:\n            return None\n        approx = cv2.minAreaRect(contornos[0])\n        angulo = approx[2]\n        if abs(angulo) == 45:\n            alto, ancho = thresh.shape[0], thresh.shape[1]\n            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)\n            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))\n            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))\n            contornos, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n            if len(contornos) == 0:\n                return None\n            approx = cv2.minAreaRect(contornos[0])\n            x = int(approx[0][0])\n            y = int(approx[0][1])\n            mitadAncho = int(approx[1][0] / 2)\n            mitadAlto = int(approx[1][1] / 2)\n            if y - mitadAlto < 0 or y + mitadAlto > imagen_rot.shape[0] or x - mitadAncho < 0 or x + mitadAncho > imagen_rot.shape[1]:\n                return None\n            rect = imagen_rot[y - mitadAlto:y + mitadAlto, x - mitadAncho:x + mitadAncho]\n            amarillo, rojo, negro, blanco = 0, 0, 0, 0\n            salida = None\n            for x in range(rect.shape[0]):\n                for y in range(rect.shape[1]):\n                    b, g, r = rect[x, y]\n                    if b > 200 and g > 200 and r > 200:\n                        blanco += 1\n                    elif b <= 1 and g <= 1 and r <= 1:\n                        negro += 1\n                    elif b > 70 and g < 5 and r > 190:\n                        rojo += 1\n                    elif b < 10 and g > 190 and r > 195:\n                        amarillo += 1\n            if (rojo + blanco) > (negro + amarillo) and rojo > blanco and blanco > negro:\n                salida = 'F'\n            elif (blanco + negro) > (amarillo + rojo) and blanco > negro:\n                salida = 'P'\n            elif (blanco + negro) > (amarillo + rojo):\n                salida = 'C'\n            elif (rojo + amarillo) > (negro + blanco):\n                salida = 'O'\n            return salida\n        return None\n\n    def procesar(self, converted_img):\n        if converted_img is None:\n            return None\n        self.img = converted_img\n        salida = None\n        if self.es_victima() is not None:\n            salida = self.devolver_letra_victimas()\n        else:\n            resultado = self.reconocer_limpiar_cartel()\n            if resultado is not None:\n                salida = self.devolver_letra_carteles()\n        return salida\n")
    __stickytape_write_module('point.py', b'import math\n\nclass Point:\n    def __init__(self, x, y):\n        self.x = x\n        self.y = y\n\n    def distance_to(self, point):\n        dx = self.x-point.x\n        dy = self.y-point.y\n        return math.sqrt(dx**2 + dy**2)\n    \n    def __str__(self) -> str:\n        return f"({self.x:.3f}, {self.y:.3f})"')
    __stickytape_write_module('utils.py', b'import math\n\ndef angle_diff(a, b):\n    clockwise = (a - b) % math.tau\n    counterclockwise = (b - a) % math.tau\n    return min(clockwise, counterclockwise)')
    from robot import Robot
    from image import ImageProcessor
    
    # Inicializar el robot y procesar la imagen obtenida
    robot = Robot()
    image_processor = ImageProcessor()  
    entrada = image_processor.salida
    while robot.step() != -1:
        img = robot.camI.getImage() 
        alto = robot.camI.getHeight()
        ancho = robot.camI.getWidth()
        robot.convertir_camara(img, alto, ancho)
    
        if not robot.hayAlgoIzquierda():
            robot.girarIzquierda90()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
            robot.avanzarBaldosa()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
        elif not robot.hayAlgoAdelante():
            robot.avanzarBaldosa()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
        elif not robot.hayAlgoDerecha():
            robot.girarDerecha90()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
            robot.avanzarBaldosa()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
        else:
            robot.girarMediaVuelta()
            robot.avanzarBaldosa()
            image_processor.procesar(robot.converted_img)
            robot.enviarMensajeVoC(entrada)
    