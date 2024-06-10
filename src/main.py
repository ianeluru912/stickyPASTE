from robot import Robot
from image import ImageProcessor

robot = Robot()
image_processor = ImageProcessor()
robot.step()
posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
baldosas_recorridas = []
baldosa_inicial = robot.positionToGrid(posicion_inicial)
baldosas_recorridas.append(baldosa_inicial)
baldosa_izquierda = robot.evaluar_baldosa_izquierda()
baldosa_derecha = robot.evaluar_baldosa_derecha()
baldosa_delantera = robot.evaluar_baldosa_delantera()
while robot.step() != -1:
    img = robot.camI.getImage() 
    alto = robot.camI.getHeight()
    ancho = robot.camI.getWidth()
    resultado = robot.convertir_camara(img, alto, ancho)
    entrada = image_processor.procesar(resultado)
    if not robot.hayAlgoIzquierda() and baldosa_izquierda not in baldosas_recorridas:
        print('baldosa izquierda no recorrida')
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
    elif not robot.hayAlgoAdelante() and baldosa_delantera not in baldosas_recorridas:
        print('baldosa delantera no recorrida')
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
    elif not robot.hayAlgoDerecha() and baldosa_delantera not in baldosas_recorridas:
        print('baldosa derecha no recorrida')
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