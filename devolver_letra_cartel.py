import cv2
def devolver_letra(img):
    gris=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh=cv2.threshold(gris, 120, 255, cv2.THRESH_BINARY)
    contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    approx=cv2.minAreaRect(contornos[0])
    angulo=approx[2]
    if abs(angulo)==45:
        alto, ancho=thresh.shape[0], thresh.shape[1]
        M=cv2.getRotationMatrix2D((ancho/2,alto/2),angulo,1)
        thresh_rot=cv2.warpAffine(thresh,M,(ancho,alto))
        imagen_rot=cv2.warpAffine(img,M,(ancho,alto))
        contornos, _ = cv2.findContours(thresh_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        approx=cv2.minAreaRect(contornos[0])
        x=int(approx[0][0])
        y=int(approx[0][1])
        mitadAncho=int(approx[1][0]/2)
        mitadAlto=int(approx[1][1]/2)
        rect=imagen_rot[y-mitadAlto:y+mitadAlto, x-mitadAncho:x+mitadAncho]    
        amarillo=0
        rojo=0
        negro=0
        blanco=0
        for x in range(rect.shape[0]):
            for y in range(rect.shape[1]):
                b, g, r=rect[x,y]
                if b>200 and g>200 and r>200:
                    blanco+=1
                elif b<=1 and g<=1 and r<=1:
                    negro+=1
                elif b>70 and g<5 and r>190:
                    rojo+=1
                elif b<10 and g>190 and r>195:
                    amarillo+=1
        print('rojo', rojo)
        print('blanco', blanco)
        print('amarillo', amarillo)
        print('negro', negro)
        if  (rojo + blanco) > (negro + amarillo) and rojo > blanco and blanco >= negro: 
            return "F"
        elif (blanco + negro) > (amarillo + rojo) and blanco > negro:
            return "P"
        elif (blanco + negro) > (amarillo + rojo): # b< 120 n <25
            return "C"
        elif (rojo + amarillo) > (negro + blanco):
            return "O"
        return None

img = cv2.imread('C:/Users/mtala/Downloads/Curso Python (Martu)/simulador/Imagenes/CD0036.png')
cartel = devolver_letra(img)
print(cartel)
