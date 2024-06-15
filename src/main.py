from robot import Robot
from image import ImageProcessor

robot = Robot()
image_processor = ImageProcessor()
robot.step()
posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
baldosas_recorridas = []
baldosa_inicial = robot.positionToGrid(posicion_inicial, robot.position)
baldosas_recorridas.append(baldosa_inicial)
while robot.step() != -1:
    candidato_izquierda = robot.coordenada_baldosa_izquierda(posicion_inicial, robot.position, robot.rotation)
    candidato_derecha = robot.coordenada_baldosa_derecha(posicion_inicial, robot.position, robot.rotation)
    candidato_adelante = robot.coordenada_baldosa_delantera(posicion_inicial, robot.position, robot.rotation)
    if not robot.hayAlgoIzquierda() and not robot.hayAlgoAdelante() and robot.hayAlgoDerecha():
        if candidato_adelante not in baldosas_recorridas and candidato_izquierda not in baldosas_recorridas or candidato_izquierda in baldosas_recorridas and candidato_adelante in baldosas_recorridas:
            print('hay dos caminos posibles')
            print('candidato adelante',candidato_adelante,'candidato izquierda', candidato_izquierda)
        else:
            robot.avanzarBaldosa()
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante() and not robot.hayAlgoDerecha() and not robot.hayAlgoIzquierda():
        if candidato_adelante not in baldosas_recorridas and candidato_izquierda not in baldosas_recorridas and candidato_derecha not in baldosas_recorridas or candidato_adelante in baldosas_recorridas and candidato_derecha in baldosas_recorridas and candidato_izquierda in baldosas_recorridas:
            print('hay tres caminos posibles')
            print('candidato adelante',candidato_adelante,'candidato izquierda', candidato_izquierda,'candidato derecha', candidato_derecha)
        elif candidato_derecha not in baldosas_recorridas and candidato_adelante not in baldosas_recorridas and candidato_izquierda in baldosas_recorridas:
            robot.avanzarBaldosa()
        else:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha() and not robot.hayAlgoAdelante() and robot.hayAlgoIzquierda():
        if candidato_adelante not in baldosas_recorridas and candidato_derecha not in baldosas_recorridas or candidato_adelante in baldosas_recorridas and candidato_derecha in baldosas_recorridas:
            print('hay dos caminos posibles')
            print('candidato adelante',candidato_adelante,'candidato derecha', candidato_derecha)
        else:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        print(baldosas_recorridas)
    elif robot.hayAlgoAdelante() and not robot.hayAlgoDerecha() and not robot.hayAlgoIzquierda():
        if candidato_derecha not in baldosas_recorridas and candidato_izquierda not in baldosas_recorridas or candidato_derecha in baldosas_recorridas and candidato_izquierda in baldosas_recorridas:
            print('hay dos caminos posibles')
            print('candidato izquierda', candidato_izquierda,'candidato derecha', candidato_derecha)
        else:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        print(baldosas_recorridas)
    if not robot.hayAlgoIzquierda():
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante():
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha():
        print('orientacion', robot.obtener_orientacion(robot.rotation))
        robot.girarDerecha90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)


