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
    radianes = robot.rotation
    candidato_izquierda = robot.coordenada_baldosa_izquierda(posicion_inicial, robot.position, radianes)
    candidato_derecha = robot.coordenada_baldosa_derecha(posicion_inicial, robot.position, radianes)
    candidato_adelante = robot.coordenada_baldosa_delantera(posicion_inicial, robot.position, radianes)
    if not robot.hayAlgoIzquierda() and not robot.hayAlgoAdelante() and robot.hayAlgoDerecha():
        print('candidato adelante',candidato_adelante,'candidato izquierda', candidato_izquierda)
        print('orientacion', robot.obtener_orientacion(radianes))
        if candidato_izquierda not in baldosas_recorridas or candidato_izquierda in baldosas_recorridas and candidato_adelante in baldosas_recorridas:
            robot.girarIzquierda90()
            robot.avanzarBaldosa()
        elif candidato_izquierda in baldosas_recorridas and candidato_adelante not in baldosas_recorridas:
            robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante() and not robot.hayAlgoDerecha() and not robot.hayAlgoIzquierda():
        print('candidato adelante',candidato_adelante,'candidato izquierda', candidato_izquierda,'candidato derecha', candidato_derecha)
        print('orientacion', robot.obtener_orientacion(radianes))
        if candidato_izquierda not in baldosas_recorridas or candidato_izquierda in baldosas_recorridas and candidato_adelante in baldosas_recorridas and candidato_derecha in baldosas_recorridas:
            robot.girarIzquierda90
            robot.avanzarBaldosa()
        elif candidato_adelante not in baldosas_recorridas:
            robot.avanzarBaldosa()
        elif candidato_derecha not in baldosas_recorridas:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
    elif not robot.hayAlgoDerecha() and not robot.hayAlgoAdelante() and robot.hayAlgoIzquierda():
        print('candidato adelante',candidato_adelante,'candidato derecha', candidato_derecha)
        print('orientacion', robot.obtener_orientacion(radianes))
        if candidato_adelante not in baldosas_recorridas or candidato_adelante in baldosas_recorridas and candidato_derecha in baldosas_recorridas:
            robot.avanzarBaldosa()
        elif candidato_derecha not in baldosas_recorridas:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
    elif robot.hayAlgoAdelante() and not robot.hayAlgoDerecha() and not robot.hayAlgoIzquierda():
        print('orientacion', robot.obtener_orientacion(radianes))
        print('candidato izquierda', candidato_izquierda,'candidato derecha', candidato_derecha)
        if candidato_izquierda not in baldosas_recorridas or candidato_izquierda in baldosas_recorridas and candidato_derecha in baldosas_recorridas:
            robot.girarIzquierda90()
            robot.avanzarBaldosa()
        elif candidato_derecha not in baldosas_recorridas:
            robot.girarDerecha90()
            robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        robot.avanzarBaldosa()
    if not robot.hayAlgoIzquierda():
        print('orientacion', robot.obtener_orientacion(radianes))
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante():
        print('orientacion', robot.obtener_orientacion(radianes))
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha():
        print('orientacion', robot.obtener_orientacion(radianes))
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
# from robot import Robot
# from image import ImageProcessor

# robot = Robot()
# image_processor = ImageProcessor()
# robot.step()
# posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
# baldosas_recorridas = []
# baldosa_inicial = robot.positionToGrid(posicion_inicial, robot.position)
# baldosas_recorridas.append(baldosa_inicial)
# while robot.step() != -1:
#     radianes = robot.rotation
#     baldosa_izquierda = robot.evaluar_baldosa_izquierda()
#     coordenada_baldosa_izquierda = robot.coordenada_baldosa_izquierda(posicion_inicial, robot.position, radianes)
#     baldosa_derecha = robot.evaluar_baldosa_derecha()
#     coordenada_baldosa_derecha = robot.coordenada_baldosa_derecha(posicion_inicial, robot.position, radianes)
#     baldosa_delantera = robot.evaluar_baldosa_delantera()
#     if not robot.hayAlgoIzquierda():
#         print('ubicacion en la grilla baldosa izquierda', coordenada_baldosa_izquierda)
#         print(radianes)
#     elif not robot.hayAlgoDerecha():
#         print('ubicacion en la grilla baldosa derecha', coordenada_baldosa_derecha)
#         print(radianes)
#     if not robot.hayAlgoIzquierda():
#         print('orientacion', robot.obtener_orientacion(radianes))
#         robot.girarIzquierda90()
#         robot.avanzarBaldosa()
#         if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
#             print('baldosa no recorrida')
#         else:
#             print('baldosa visitada')
#         print(baldosas_recorridas)
#     elif not robot.hayAlgoAdelante():
#         print('orientacion', robot.obtener_orientacion(radianes))
#         robot.avanzarBaldosa()
#         if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
#             print('baldosa no recorrida')
#         else:
#             print('baldosa visitada')
#         print(baldosas_recorridas)
#     elif not robot.hayAlgoDerecha():
#         print('orientacion', robot.obtener_orientacion(radianes))
#         robot.girarDerecha90()
#         robot.avanzarBaldosa()
#         if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
#             print('baldosa no recorrida')
#         else:
#             print('baldosa visitada')
#         print(baldosas_recorridas)
#     else:
#         robot.girarMediaVuelta()
#         robot.avanzarBaldosa()
#         if robot.isVisited(baldosas_recorridas, posicion_inicial, robot.position) is not True:
#             print('baldosa no recorrida')
#         else:
#             print('baldosa visitada')
#         print(baldosas_recorridas)

