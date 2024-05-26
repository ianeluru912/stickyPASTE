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
