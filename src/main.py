from robot import Robot
from image import ImageProcessor

robot = Robot()
image_processor = ImageProcessor()
robot.step()
posicion_inicial = {'x': robot.position.x, 'y': robot.position.y}
baldosas_recorridas = []
baldosa_inicial = robot.positionToGrid(posicion_inicial)
baldosas_recorridas.append(baldosa_inicial)
bifurcaciones = []

while robot.step() != -1:
    baldosa_izquierda = robot.evaluar_baldosa_izquierda()
    baldosa_derecha = robot.evaluar_baldosa_derecha()
    baldosa_delantera = robot.evaluar_baldosa_delantera()
    baldosa_trasera = robot.evaluar_baldosa_trasera()
    direccion = None
    if not robot.hayAlgoIzquierda() and not robot.hayAlgoAdelante(): # posibles caminos adelante o a la izquierda
        print('bifurcacion delantera', robot.positionToGridBaldosasEspecificas(baldosa_delantera, posicion_inicial))
        bifurcaciones.append(robot.positionToGrid(posicion_inicial))
        print('bifurcaciones:', bifurcaciones)
        baldosa_trasera = robot.evaluar_baldosa_trasera()
        direccion = 'adelante'
        robot.recorrer_bifurcaciones(posicion_inicial, bifurcaciones, direccion, baldosa_trasera)
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoIzquierda() and not robot.hayAlgoDerecha() and robot.hayAlgoAdelante(): # posibles caminos derecha e izquierda, frente bloqueado
        print('bifurcacion derecha:', robot.positionToGridBaldosasEspecificas(baldosa_derecha, posicion_inicial))
        bifurcaciones.append(robot.positionToGrid(posicion_inicial))
        print('bifurcaciones:', bifurcaciones)
        baldosa_trasera = robot.evaluar_baldosa_trasera()
        direccion = 'derecha'
        robot.recorrer_bifurcaciones(posicion_inicial, bifurcaciones, direccion, baldosa_trasera)
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoDerecha() and not robot.hayAlgoAdelante(): # posibles caminos adelante o a la derecha
        print('bifurcacion derecha:', robot.positionToGridBaldosasEspecificas(baldosa_derecha, posicion_inicial))
        bifurcaciones.append(robot.positionToGrid(posicion_inicial))
        print('bifurcaciones:', bifurcaciones)
        baldosa_trasera = robot.evaluar_baldosa_trasera()
        direccion = 'derecha'
        robot.recorrer_bifurcaciones(posicion_inicial, bifurcaciones, direccion, baldosa_trasera)
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
    elif not robot.hayAlgoIzquierda() and not robot.hayAlgoDerecha(): # posibles caminos derecha e izquierda, frente bloqueado
        print('bifurcacion derecha:', robot.positionToGridBaldosasEspecificas(baldosa_derecha, posicion_inicial))
        bifurcaciones.append(robot.positionToGrid(posicion_inicial))
        print('bifurcaciones:', bifurcaciones)
        baldosa_trasera = robot.evaluar_baldosa_trasera()
        direccion = 'derecha'
        robot.recorrer_bifurcaciones(posicion_inicial, bifurcaciones, direccion, baldosa_trasera)
        robot.girarIzquierda90()
        robot.avanzarBaldosa()
        if robot.isVisited(baldosas_recorridas, posicion_inicial) is not True:
            print('baldosa no recorrida')
        else:
            print('baldosa visitada')
        print(baldosas_recorridas)
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



