from robot import Robot
from image import ImageProcessor

# Inicializar el robot y procesar la imagen obtenida
robot = Robot()
image_processor = ImageProcessor()
# posicion_inicial = []
robot.step()
posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
# dict(posicion_inicial)
# Obtener lo leido de la camara
while robot.step() != -1:
    img = robot.camI.getImage() 
    alto = robot.camI.getHeight()
    ancho = robot.camI.getWidth()
    resultado = robot.convertir_camara(img, alto, ancho)
    ubicacion_en_la_grilla = robot.positionToGrid({'x': posicion_inicial['x'], 'y': posicion_inicial['y']})
    baldosas_recorridas = ()
    # Procesar la imagen y determinar la letra o el tipo de cartel/victima
    entrada = image_processor.procesar(resultado)
    
    if not robot.hayAlgoIzquierda():
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.girarIzquierda90()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    elif not robot.hayAlgoAdelante():
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            print(robot.enviarMensajeVoC(entrada))
    elif not robot.hayAlgoDerecha():
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.girarDerecha90()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    else:
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.girarMediaVuelta()
        if ubicacion_en_la_grilla not in baldosas_recorridas:
            print('baldosa desconocida')
        robot.avanzarBaldosa()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            print(robot.enviarMensajeVoC(entrada))
