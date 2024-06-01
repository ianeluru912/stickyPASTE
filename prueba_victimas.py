import cv2
import numpy as np

analizar='C:/Users/mtala/Downloads/Curso Python (Martu)/simulador/Imagenes/CI0027.png'
imagen=cv2.imread(analizar)
cv2.imshow("Original", imagen)

# Comprimimos en unas pocas líneas lo que hicimos en el programa anterior
gris=cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gris", gris)

ret, thresh=cv2.threshold(gris, 140, 255, cv2.THRESH_BINARY)
cv2.imshow("Thresh", thresh)

contornos, jerarquia = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(imagen, contornos, -1, (0, 0, 255), 2)
cv2.imshow("Contornos", imagen)

print(len(contornos[0]))
# Ahora, si tenemos un único contorno y tiene pocos puntos, queremos obtener el rectángulo que contiene el cartel sin nada alrededor
# minAreaRect es una función que nos permite obtener el rectángulo más grande que encuentre en los contornos
# Me devuelve ((xCentro,yCentro), (ancho,alto), angulo)

if len(contornos)==1 and len(contornos[0])<=10: # Si tenemos un único contorno y tiene pocos puntos, es un rectángulo
    approx=cv2.minAreaRect(contornos[0])
    angulo=approx[2] # Vamos a ver si no detectó un ángulo rotado
    print(f"El ángulo del rectángulo es: {angulo}")
    if angulo%90==0: # Si el ángulo es múltiplo de 90 (no lo tengo rotado)
        x=int(approx[0][0])
        y=int(approx[0][1])
        mitadAncho=int(approx[1][0]/2)
        mitadAlto=int(approx[1][1]/2)    
        # Voy a recortar de la matriz (una imagen es una matriz) la parte del cartel que me interesa
        
        rect=thresh[y-mitadAlto:y+mitadAlto, x-mitadAncho:x+mitadAncho] # Recordar que x e y están en el centro del rectángulo

        print(f"El tamaño del cartel es: {rect.shape}")
        cv2.imshow("Cartel", rect) 

        # Cómo saber cuál es la letra?
        # a) ¿Qué porcentaje de la imagen (o parte de la imagen) está ocupada por la letra?

        tamanio=rect.shape[0]*rect.shape[1]

        pixelesNegros=np.count_nonzero(rect==0)
        
        print(f"Tamaño: {tamanio} - pixelesNegros: {pixelesNegros} - Porcentaje: {pixelesNegros/tamanio}")
        # b) Divido en regiones el cartel, ¿cuáles tienen algún pixel en negro?
        
        cuadritoArriba=thresh[y-mitadAlto:y-int(mitadAlto/3), x-int(mitadAncho/3):x+int(mitadAncho/3)]
        cuadritoAbajo=thresh[y+int(mitadAlto/3):y+mitadAlto, x-int(mitadAncho/3):x+int(mitadAncho/3)]
        
        cv2.imshow("Cuadrito arriba", cuadritoArriba)
        cv2.imshow("Cuadrito abajo", cuadritoAbajo)
cv2.waitKey(0)