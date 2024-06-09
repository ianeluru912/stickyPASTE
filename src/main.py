from robot import Robot
from image import ImageProcessor

robot = Robot()
image_processor = ImageProcessor()
robot.step()
posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
baldosas_recorridas = []
baldosa_inicial = robot.positionToGrid(posicion_inicial)
baldosas_recorridas.append(baldosa_inicial)
while robot.step() != -1:
    img = robot.camI.getImage() 
    alto = robot.camI.getHeight()
    ancho = robot.camI.getWidth()
    resultado = robot.convertir_camara(img, alto, ancho)
    # ubicacion_en_la_grilla = robot.positionToGrid({'x': posicion_inicial['x'], 'y': posicion_inicial['y']})
    entrada = image_processor.procesar(resultado)
    if not robot.hayAlgoIzquierda():
        robot.girarIzquierda90()
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
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
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            robot.enviarMensajeVoC(entrada)
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
        resultado = robot.convertir_camara(robot.camI.getImage(), alto, ancho)
        entrada = image_processor.procesar(resultado)
        if entrada is not None:
            print(robot.enviarMensajeVoC(entrada))