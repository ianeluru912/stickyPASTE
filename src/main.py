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
    baldosa_izquierda = robot.evaluar_baldosa_izquierda()
    coordenada_baldosa_izquierda = robot.coordenada_baldosa_izquierda(posicion_inicial)
    baldosa_derecha = robot.evaluar_baldosa_derecha()
    coordenada_baldosa_derecha = robot.coordendada_baldosa_derecha(posicion_inicial)
    baldosa_delantera = robot.evaluar_baldosa_delantera()
    if not robot.hayAlgoIzquierda():
        print('ubicacion en la grilla baldosa izquierda', coordenada_baldosa_izquierda)
    elif not robot.hayAlgoDerecha():
        print('ubicacion en la grilla baldosa derecha', coordenada_baldosa_derecha)
    if not robot.hayAlgoIzquierda():
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoAdelante():
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha():
        robot.girarDerecha90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    else:
        robot.girarMediaVuelta()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)