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
    if not robot.hayAlgoIzquierda and not robot.hayAlgoAdelante: # posibles caminos adelante o a la izquierda
        print('bifurcacion delantera', robot.positionToGridBaldosasEspecificas(baldosa_delantera))
    elif not robot.hayAlgoIzquierda and not robot.hayAlgoDerecha and robot.hayAlgoAdelante: # posibles caminos derecha e izquierda, frente bloqueado
        print('bifurcacion derecha 1:', robot.positionToGridBaldosasEspecificas(baldosa_derecha))
    elif not robot.hayAlgoDerecha and not robot.hayAlgoAdelante: # posibles caminos adelante o a la derecha
        print('bifurcacion derecha 2:', robot.positionToGridBaldosasEspecificas(baldosa_derecha))
    elif not robot.hayAlgoDerecha and not robot.hayAlgoAdelante: # posibles caminos adelante o a la derecha
        print('bifurcacion derecha 3:', robot.positionToGridBaldosasEspecificas(baldosa_derecha))
    elif not robot.hayAlgoIzquierda():
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




