import cv2
import math
import utils
import numpy as np
from point import Point
from datetime import datetime


class ImageProcessor:
    def __init__(self):
        self.img = None
        self.salida = None
        self.lastTokenPosition = Point(10000, 10000)
        self.lastTokenRotation = 150.8
        self.lastCamera = 'j'

    def debugShow(self, image):
        cv2.imshow("V", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def es_victima(self):
        if self.img is None or self.img.size == 0:
            return None
        
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours if len(contours) == 1 and len(contours[0]) <= 10 else None
    
    def devolver_letra_victimas(self):
        self.salida = None
        recorte=0 # Este recorte lo vamos a utilizar cuando realizamos una transformación del cartel deformado
        # para sacarle algunos bordes negritos que nos quedan

        
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 160, 255, cv2.THRESH_BINARY)
        paraMostrarDespues=thresh.copy()
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return self.salida


        if len(contours) == 1:
            if len(contours[0]) > 10:
                # print("Encontré un cartel que parece una letra pero tiene demasiados puntos su contorno")

                # ¿Tiene thresh una cantidad de pixeles blancos razonables?
                # pixeles_blancos = np.count_nonzero(thresh == 255)
                # tamanio = thresh.shape[0] * thresh.shape[1]
                # porcentaje_blancos = pixeles_blancos / tamanio
                # print("Porcentajes")
                # print(pixeles_blancos, tamanio, porcentaje_blancos)
                # if porcentaje_blancos < 0.05: #Si tengo pocos blancos me voy
                #     return None
                


                # get the convex hull
                hull = cv2.convexHull(contours[0])
                # get the corners
                oriPoints = cv2.approxPolyDP(hull, 0.01*cv2.arcLength(hull, True), True)
                if(len(oriPoints) != 4):
                    # print("No es un cuadrado")
                    return self.salida

                else:
                    # get the min value of x in oriPoints
                    min_x = min(oriPoints[:,0,0])
                    # get the max value of x in oriPoints
                    max_x = max(oriPoints[:,0,0])
                    # get the min value of y in oriPoints
                    min_y = min(oriPoints[:,0,1])
                    # get the max value of y in oriPoints
                    max_y = max(oriPoints[:,0,1])
                    #Vamos a tratar de que el cartel quede cuadrado en la transformación
                    # Me fijo si lo agrando de ancho o de alto
                    difx = max_x - min_x
                    dify = max_y - min_y
                    plusx=0
                    plusy=0
                    # calculo la mitad de lo que tengo que estirarlo, así lo corro un poquito menos en el mínimo y un poquito mas en máximo
                    if difx > dify:
                        plusy = (difx - dify) / 2
                    else:
                        plusx = (dify - difx) / 2

                    dstPoints = np.array([[min_x-plusx, min_y-plusy], [max_x+plusx, min_y-plusy], [max_x+plusx, max_y+plusy], [min_x-plusx, max_y+plusy]], dtype=np.float32)

                    oriPoints = oriPoints.reshape(4, 2)
                    oriPoints = oriPoints.astype(np.float32)

                    oriPoints= utils.sortCw(oriPoints)
                    dstPoints= utils.sortCw(dstPoints)
                    
                    minXPoint = oriPoints[0]
                    maxXPoint = oriPoints[1]
                    # print("Llegué a calcular los puntos")
                    M = cv2.getPerspectiveTransform(oriPoints, dstPoints)
                    thresh = cv2.warpPerspective(thresh, M, (64, 64))
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


                    # print("Quedó de ", len(contours), len(contours[0]))
                    recorte=2 # cantidad de pixels que recorto de cada lado para sacar bordes negros
                    # print("MinXPoint: ",minXPoint)
                    # print("MaxXPoint: ",maxXPoint)
            else:
                pass
                # print("Encontró una letra copada sin tener que transformarla")

            if len(contours) == 1 and len(contours[0]) <=40:    
                
                approx = cv2.minAreaRect(contours[0])
                angulo = approx[2]
                if angulo % 90 == 0:
                    # print("Encontré un cartel que parece una letra")
                    x = int(approx[0][0])
                    y = int(approx[0][1])                
                    if x < 26 or x > 38:
                        # print("CHAU porque x no está en el rango", x) 
                        return None
                    if y < 26 or y > 38: 
                        # print("CHAU porque y no está en el rango")
                        return None
                    mitad_ancho = int(approx[1][0] / 2)-recorte
                    mitad_alto = int(approx[1][1] / 2)-recorte
                    
                    rect = thresh[y - mitad_alto:y + mitad_alto, x - mitad_ancho:x + mitad_ancho]
                    tamanio = rect.shape[0] * rect.shape[1]
                    if tamanio == 0: 
                        # print("Me dio tamaño 0")
                        return None
                    if abs(rect.shape[0] - rect.shape[1]) > 2:
                        # print("CHAU porque no es un cuadrado")
                        return None
 
                    
                    pixeles_negros = np.count_nonzero(rect == 0)
                    if pixeles_negros == 0: 
                        # print("CHAU porque no hay pixeles negros")
                        return None
                    
                    porcentaje_negros = pixeles_negros / tamanio
                    if porcentaje_negros < 0.1:
                        # print("CHAU porque hay muy pocos porcentaje de negros")
                        return None
                    
                    
                    cuadritoArriba = thresh[y - mitad_alto:y - int(mitad_alto / 3), x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]
                    cuadritoAbajo = thresh[y + int(mitad_alto / 3):y + mitad_alto, x - int(mitad_ancho / 3):x + int(mitad_ancho / 3)]
                    top_central = y - int(mitad_alto / 3)
                    bottom_central = y + int(mitad_alto / 3)
                    left_central = x - int(mitad_ancho / 3)
                    right_central = x + int(mitad_ancho / 3)
                    cuadritoCentral = thresh[top_central:bottom_central, left_central:right_central]
                    pixeles_negros_central = np.count_nonzero(cuadritoCentral == 0)
                    pixeles_negros_arriba = np.count_nonzero(cuadritoArriba == 0)
                    pixeles_negros_abajo = np.count_nonzero(cuadritoAbajo == 0)

                    if pixeles_negros_abajo <= 3 and pixeles_negros_arriba <= 6 and pixeles_negros_central >= 30: #ACAACA decía 35 lo relajamos
                        self.salida = 'H'
                    elif pixeles_negros_abajo >= 13 and pixeles_negros_arriba >= 13:
                        self.salida = 'S'
                    elif pixeles_negros_abajo >= 15 and pixeles_negros_arriba <= 8: #ACAACA Antes decía 5, lo relajamos
                        self.salida = 'U'
                    elif pixeles_negros_abajo >= 1 and pixeles_negros_arriba >= 1:
                        return self.salida
                    # print("Pixeles")
                    # print(pixeles_negros_arriba, pixeles_negros_central, pixeles_negros_abajo)
                    # print(self.salida)
                    # # Descomentar para ver si hay falsos positivos
                    # self.debugShow(self.img)
                    # self.debugShow(paraMostrarDespues)
                    # self.debugShow(rect)
            return self.salida
        
    def reconocer_limpiar_cartel(self):
        if self.img is None or self.img.size == 0:
            return None
        
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None
        
            
        approx = cv2.minAreaRect(contours[0])
        angulo = approx[2]
        if abs(angulo)%45 == 0:
            alto, ancho = thresh.shape[0], thresh.shape[1]
            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)
            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))
            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))
            contours, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return None
            
            x = int(approx[0][0])
            y = int(approx[0][1])
            mitad_ancho = int(approx[1][0] / 2)
            mitad_alto = int(approx[1][1] / 2)

            if y - mitad_alto < 0 or y + mitad_alto > imagen_rot.shape[0] or x - mitad_ancho < 0 or x + mitad_ancho > imagen_rot.shape[1]:
                return None
            rect = imagen_rot[y - mitad_alto:y + mitad_alto, x - mitad_ancho:x + mitad_ancho]
            return rect, True
        return None
    
    def devolver_letra_carteles(self):
        self.salida = None
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)
        contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contornos) == 0:
            return None
        
        area=cv2.contourArea(contornos[0])
        # print("Area: ", area)
        if area< 100:
            return None
        
        approx = cv2.minAreaRect(contornos[0])
        angulo = approx[2]
        if abs(angulo) == 45:
            alto, ancho = thresh.shape[0], thresh.shape[1]
            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)
            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))
            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))
            contornos, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contornos) == 0:
                return None
            
            x = int(approx[0][0])
            y = int(approx[0][1])
            
            if x < 26 or x > 38: return None
            if y < 26 or y > 38: return None

            mitadAncho = int(approx[1][0] / 2)
            mitadAlto = int(approx[1][1] / 2)
            
            if y - mitadAlto < 0 or y + mitadAlto > imagen_rot.shape[0] or x - mitadAncho < 0 or x + mitadAncho > imagen_rot.shape[1]:
                return None
            rect = imagen_rot[y - mitadAlto:y + mitadAlto, x - mitadAncho:x + mitadAncho]
            
            tamanio = rect.shape[0] * rect.shape[1]
            if tamanio == 0: return None
                       
            if abs(rect.shape[0] - rect.shape[1]) > 2:
                #print("CHAU porque no es un cuadrado")
                return None

            amarillo, rojo, negro, blanco = 0, 0, 0, 0
            for x in range(rect.shape[0]):
                for y in range(rect.shape[1]):
                    pixel = rect[x, y]
                    b, g, r = pixel[:3]
                    if b > 200 and g > 200 and r > 200:
                        blanco += 1
                    elif b <= 1 and g <= 1 and r <= 1:
                        negro += 1
                    elif b > 70 and g < 5 and r > 190:
                        rojo += 1
                    elif b < 10 and g > 190 and r > 195:
                        amarillo += 1
            if rojo > 0 and rojo > blanco and rojo > negro and rojo > amarillo and blanco == 0 and negro == 0 and amarillo == 0:
                self.salida = 'F'
            elif (blanco + negro) > (amarillo + rojo) and blanco > negro:
                self.salida = 'P'
            elif (blanco + negro) > (amarillo + rojo):
                self.salida = 'C'
            elif rojo > 0 and amarillo > 0 and rojo > blanco and rojo > negro and rojo > amarillo and amarillo > blanco and amarillo > negro:
                self.salida = 'O'
            return self.salida
        
    def procesar(self, converted_img, lastPosition, lastRotation, camera):
        # if converted_img is None or converted_img.size == 0:
        #     return None
        # # si es la misma cámara, se movió y rotó poquito: None!!
        # print(utils.normalizacion_radianes(self.lastTokenRotation - lastRotation))
        if camera == self.lastCamera and lastPosition.distance_to(self.lastTokenPosition) < 0.015 and utils.normalizacion_radianes(self.lastTokenRotation - lastRotation) < math.pi/8:
            # print('no analizo', lastPosition.distance_to(self.lastTokenPosition))
            return None
        # si no, antes de procesar, guardamos la última, cámara, 
        # #posición y rotación, luego procesamos

        self.img = converted_img
        victima = self.devolver_letra_victimas()
        if victima is not None:
            # print('letra', victima, lastPosition)
            self.lastTokenPosition = lastPosition
            self.lastTokenRotation = lastRotation
            self.lastCamera = camera
            return victima
        else:
            cartel = self.reconocer_limpiar_cartel()
            if cartel is not None:
                salida = self.devolver_letra_carteles()
                if salida is not None:
                    # print('salida', salida, lastPosition)
                    self.lastTokenPosition = lastPosition
                    self.lastTokenRotation = lastRotation
                    self.lastCamera = camera
                return salida
        return None
    
    def see_hole(self, img):
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mitad = gris[43:, :]
        tamanio = mitad.shape[0] * mitad.shape[1]
        pixeles_negros = np.count_nonzero(mitad < 31)
        porcentaje_negros = pixeles_negros / tamanio
        black_Hole = False
        if porcentaje_negros >=0.85 and porcentaje_negros <= 0.97:
            black_Hole = True
        return black_Hole
