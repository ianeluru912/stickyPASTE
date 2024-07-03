from robot import Robot
from point import Point
import cv2
from math import pi as PI

from visualization import MapVisualizer

robot = Robot()

mapvis = MapVisualizer()
robot.step()
inicio=robot.map.getTileAt(0, 0)
inicio.set_area(1)
while robot.step() != -1:
    print("Loop")
    # 0. Actualizar mapa
    robot.updateMap()
    robot.map.writeMap("map.txt", robot)
    mapvis.send_map(robot.map)    

    # 1. Dame baldosas vecinas candidatas a donde puedo moverme
    tiles = robot.checkNeighbours()
    
    # 2. Elegir una baldosa de las candidatas
    tiles.sort(key=lambda t: t.visits)
    tile = tiles[0]

    # 3. Mover robot a baldosa elegida
    robot.moveToTile(tile)
    # print all tiles that are valids
    for t in robot.map.tiles:
        print(t.col,t.row, t.is_valid())
    # print(robot.bh_ahead())
    # print(robot.imageProcessor.see_hole(robot.convertir_camara(robot.camD.getImage(), 64, 64)))
    # 4. Repetir
    
# print(robot.position)
# destino=Point(robot.position.x,robot.position.y+0.06)
# robot.moveToPoint(destino)
# print(robot.position)
# print(robot.lidar.get_walls_2(robot.rotation))
# print('---')
# destino=Point(robot.position.x,robot.position.y+0.18)
# robot.moveToPoint(destino)
# print(robot.position)
# print(robot.lidar.get_walls_2(robot.rotation))
# print('---')
# destino=Point(robot.position.x+0.18,robot.position.y)
# robot.moveToPoint(destino)
# destino=Point(robot.position.x,robot.position.y-0.12)
# robot.moveToPoint(destino)
# print(robot.position)
    # print(robot.lidar.ver_walls(robot.rotation))
    # print(robot.lidar.get_walls_4(robot.rotation))
    # print('---')
    # print(robot.lidar.ver_walls(robot.rotation))
    # print('---')

# destino=Point(robot.position.x+0.36,robot.position.y)
# robot.moveToPoint(destino)
# print(robot.position)
# robot.girar(PI/3)
# print(robot.lidar.get_walls_2(robot.rotation))
# destino=Point(robot.position.x-0.12,robot.position.y)
# robot.moveToPoint(destino)
# print(robot.position)
# print(robot.lidar.get_walls_2(robot.rotation))
# destino=Point(robot.position.x,robot.position.y+0.12)
# robot.moveToPoint(destino)
# print(robot.position)
# print(robot.lidar.get_walls_2(robot.rotation))

# print(robot.rotation)
# print(robot.lidar.rotToLidar(robot.rotation))
# robot.girar(PI/2)
# print(robot.rotation)
# print(robot.lidar.rotToLidar(robot.rotation))
