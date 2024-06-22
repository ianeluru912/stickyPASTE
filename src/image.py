import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.img = None
        self.salida = None
    def es_victima(self):
        if self.img is None or self.img.size == 0:
            return None
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contorno_img = np.copy(self.img)
        for cont in contours:
            cv2.drawContours(contorno_img, [cont], -1, (0, 255, 0), 2)
        return contours if len(contours) == 1 and len(contours[0]) <= 10 else None
    def devolver_letra_victimas(self):
        self.salida = None
        gris = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return self.salida
        thresh_img = np.copy(thresh)
        for cont in contours:
            cv2.drawContours(thresh_img, [cont], -1, (0, 255, 0), 2)
        if len(contours) == 1 and len(contours[0]) <= 10:
            approx = cv2.minAreaRect(contours[0])
            angulo = approx[2]
            if angulo % 90 == 0: #si
                x = int(approx[0][0])
                y = int(approx[0][1])
                mitad_ancho = int(approx[1][0] / 2)
                mitad_alto = int(approx[1][1] / 2)
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
                if pixeles_negros_abajo <= 3 and pixeles_negros_arriba <= 6 and pixeles_negros_central >= 35:
                    self.salida = 'H'
                elif pixeles_negros_abajo >= 13 and pixeles_negros_arriba >= 13:
                    self.salida = 'S'
                elif pixeles_negros_abajo >= 15 and pixeles_negros_arriba <= 5:
                    self.salida = 'U'
                elif pixeles_negros_abajo >= 1 and pixeles_negros_arriba >= 1:
                    return self.salida
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
        if abs(angulo) == 45:
            alto, ancho = thresh.shape[0], thresh.shape[1]
            M = cv2.getRotationMatrix2D((ancho / 2, alto / 2), angulo, 1)
            thresh_rot = cv2.warpAffine(thresh, M, (ancho, alto))
            imagen_rot = cv2.warpAffine(self.img, M, (ancho, alto))
            contours, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return None
            approx = cv2.minAreaRect(contours[0])
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
            approx = cv2.minAreaRect(contornos[0])
            x = int(approx[0][0])
            y = int(approx[0][1])
            mitadAncho = int(approx[1][0] / 2)
            mitadAlto = int(approx[1][1] / 2)
            if y - mitadAlto < 0 or y + mitadAlto > imagen_rot.shape[0] or x - mitadAncho < 0 or x + mitadAncho > imagen_rot.shape[1]:
                return None
            rect = imagen_rot[y - mitadAlto:y + mitadAlto, x - mitadAncho:x + mitadAncho]
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
    def procesar(self, converted_img):
        # if converted_img is None or converted_img.size == 0:
        #     return None
        salida = None
        self.img = converted_img
        victima = self.devolver_letra_victimas()
        if victima is not None:
            print('letra', victima)
            return victima
        else:
            cartel = self.reconocer_limpiar_cartel()
            if cartel is not None:
                salida = self.devolver_letra_carteles()
                print('cartel', salida)
                return salida
        return salida
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
