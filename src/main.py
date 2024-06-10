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
    imgIzq = robot.camI.getImage()
    altoIzq = robot.camI.getHeight()
    anchoIzq = robot.camI.getWidth()
    imgDer = robot.camD.getImage()
    altoDer = robot.camD.getHeight()
    anchoDer = robot.camD.getWidth()
    resultadoIzq = robot.convertir_camara(imgIzq, altoIzq, anchoIzq)
    resultadoDer = robot.convertir_camara(imgDer, altoDer, anchoDer)
    entradaIzq = image_processor.procesar(resultadoIzq)
    entradaDer = image_processor.procesar(resultadoDer)
    if not robot.hayAlgoIzquierda() and baldosa_izquierda not in baldosas_recorridas:
        print('baldosa izquierda no recorrida')
        robot.girarIzquierda90()
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
            resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
            resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
            entradaIzq = image_processor.procesar(resultadoIzq)
            entradaDer = image_processor.procesar(resultadoDer)
            if entradaIzq is not None:
                robot.enviarMensajeVoC(entradaIzq)
            if entradaDer is not None:
                robot.enviarMensajeVoC(entradaDer)
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante() and baldosa_delantera not in baldosas_recorridas:
        print('baldosa delantera no recorrida')
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
            resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
            resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
            entradaIzq = image_processor.procesar(resultadoIzq)
            entradaDer = image_processor.procesar(resultadoDer)
            if entradaIzq is not None:
                robot.enviarMensajeVoC(entradaIzq)
            if entradaDer is not None:
                robot.enviarMensajeVoC(entradaDer)
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha() and baldosa_delantera not in baldosas_recorridas:
        print('baldosa derecha no recorrida')
        robot.girarDerecha90()
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
            resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
            resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
            entradaIzq = image_processor.procesar(resultadoIzq)
            entradaDer = image_processor.procesar(resultadoDer)
            if entradaIzq is not None:
                robot.enviarMensajeVoC(entradaIzq)
            if entradaDer is not None:
                robot.enviarMensajeVoC(entradaDer)
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    else:
        robot.girarMediaVuelta()
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
            resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
            resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
            entradaIzq = image_processor.procesar(resultadoIzq)
            entradaDer = image_processor.procesar(resultadoDer)
            if entradaIzq is not None:
                robot.enviarMensajeVoC(entradaIzq)
            if entradaDer is not None:
                robot.enviarMensajeVoC(entradaDer)
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)