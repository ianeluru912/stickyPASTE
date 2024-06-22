from robot import Robot
from point import Point
import cv2
robot = Robot()

while robot.step() != -1:
    # 0. Actualizar mapa
    print(robot.pantano_ahead())
    print(robot.bh_ahead())
    print(robot.verde_ahead())
    print(robot.rojo_ahead())
    print(robot.azul_ahead())
    print(robot.violeta_ahead())
    # print(robot.suelo_ahead())
    print(robot.checkpoint_ahead())
    # robot.updateMap()
    # robot.map.writeMap("map.txt", robot)

    # # 1. Dame baldosas vecinas candidatas a donde puedo moverme
    # tiles = robot.checkNeighbours()
    
    # # # 2. Elegir una baldosa de las candidatas
    # tiles.sort(key=lambda t: t.visits)
    # tile = tiles[0]

    # # 3. Mover robot a baldosa elegida
    # robot.moveToTile(tile)
    # # print(robot.bh_ahead())
    # print(robot.imageProcessor.see_hole(robot.convertir_camara(robot.camD.getImage(), 64, 64)))
    # 4. Repetir

