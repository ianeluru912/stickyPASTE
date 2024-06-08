from robot import Robot
from image import ImageProcessor

# Inicializar el robot y procesar la imagen obtenida
robot = Robot()
image_processor = ImageProcessor()

# Obtener lo leído de la cámara
while robot.step() != -1:
    img = robot.camI.getImage() 
    alto = robot.camI.getHeight()
    ancho = robot.camI.getWidth()
    resultado = robot.convertir_camara(img, alto, ancho)
    
    # Procesar la imagen y determinar la letra o el tipo de cartel/víctima
    entrada = image_processor.procesar(resultado)
    
    if not robot.hayAlgoIzquierda():
        robot.girarIzquierda90()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            print(robot.enviarMensajeVoC(entrada))
    elif not robot.hayAlgoDerecha():
        robot.girarDerecha90()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            print(robot.enviarMensajeVoC(entrada))

# from robot import Robot
# from image import ImageProcessor

# # Inicializar el robot y procesar la imagen obtenida
# robot = Robot()
# image_processor = ImageProcessor()  

# # Obtener lo leido de la camara -

# # entrada = image_processor.procesar(robot.img_a_convertir, image_processor.letra_img)

# while robot.step() != -1:
#     img = robot.camI.getImage() 
#     alto = robot.camI.getHeight()
#     ancho = robot.camI.getWidth()
#     resultado = robot.convertir_camara(img, alto, ancho)
#     entrada = image_processor.procesar(resultado) 
#     # color = robot.detectar_color(r, g, b)
#     if not robot.hayAlgoIzquierda():
#         robot.girarIzquierda90() 
#         image_processor.procesar(resultado)
#         if entrada is not None:
#             print(robot.enviarMensajeVoC(entrada))
#         robot.avanzarBaldosa()
#         image_processor.procesar(resultado)
#         if entrada is not None:
#             print(robot.enviarMensajeVoC(entrada))
#     elif not robot.hayAlgoAdelante():
#         robot.avanzarBaldosa()
#         image_processor.procesar(resultado)
#         if entrada is not None: 
#             print(robot.enviarMensajeVoC(entrada))
#     elif not robot.hayAlgoDerecha():
#         robot.girarDerecha90()
#         image_processor.procesar(resultado)
#         if entrada is not None:
#             print(robot.enviarMensajeVoC(entrada))
#         robot.avanzarBaldosa()
#         image_processor.procesar(resultado)
#         if entrada is not None:
#             print(robot.enviarMensajeVoC(entrada))
#     else:
#         robot.girarMediaVuelta()
#         robot.avanzarBaldosa()
#         image_processor.procesar(resultado)
#         if entrada is not None:
#             print(robot.enviarMensajeVoC(entrada))
