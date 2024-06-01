import cv2
import numpy as np

# SOLO FALLA CON LA IMG CI0022 (no anda con ningún valor thresh)
def es_cartel(img_ingresada):
    gris = cv2.cvtColor(img_ingresada, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gris, 100, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contorno_img = np.copy(img_ingresada)
    for cont in contours:
        cv2.drawContours(contorno_img, [cont], -1, (0, 255, 0), 2)
    return contours if len(contours) == 1 and len(contours[0]) <= 10 else None
img_ingresada = cv2.imread('C:/Users/mtala/OneDrive/Escritorio/scr_repo/stickyPASTE/Imagenes/CI0091.png')
resultado = es_cartel(img_ingresada)
print('array: ', resultado)