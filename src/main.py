from robot import Robot
from image import ImageProcessor

robot = Robot()
image_processor = ImageProcessor()

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
    
    if not robot.hayAlgoIzquierda():
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
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
    elif not robot.hayAlgoDerecha():
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
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()
        resultadoIzq = robot.convertir_camara(robot.camI.getImage(), altoIzq, anchoIzq)
        resultadoDer = robot.convertir_camara(robot.camD.getImage(), altoDer, anchoDer)
        entradaIzq = image_processor.procesar(resultadoIzq)
        entradaDer = image_processor.procesar(resultadoDer)
        if entradaIzq is not None:
            robot.enviarMensajeVoC(entradaIzq)
        if entradaDer is not None:
            robot.enviarMensajeVoC(entradaDer)