from robot import Robot
from image import ImageProcessor

# Inicializar el robot y procesar la imagen obtenida
robot = Robot()
image_processor = ImageProcessor()  

# Obtener lo leido de la camara -

nroImagen = 0

while robot.step() != -1:
    robot.grabar(nroImagen)
    nroImagen += 1
    
    if not robot.hayAlgoIzquierda():
        robot.girarIzquierda90()
        robot.grabar(nroImagen)
        nroImagen += 1
        robot.avanzarBaldosa()
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
    elif not robot.hayAlgoDerecha():
        robot.girarDerecha90()
        robot.grabar(nroImagen)
        nroImagen += 1
        robot.avanzarBaldosa()
    else:
        robot.girarMediaVuelta()
        robot.grabar(nroImagen)
        nroImagen += 1
        robot.avanzarBaldosa()


