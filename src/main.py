from robot import Robot
from point import Point
import cv2
from math import pi as PI
from map import TileType

robot = Robot()

robot.step()
inicio=robot.map.getTileAt(0, 0)
inicio.set_area(1)
inicio.type=TileType.STARTING
while robot.step() != -1:
    print("Loop")
    # 0. Actualizar mapa
    robot.updateMap() 

    # 1. Dame baldosas vecinas candidatas a donde puedo moverme
    tiles = robot.checkNeighbours()
    
    # 2. Elegir una baldosa de las candidatas
    tiles.sort(key=lambda t: t.visits)
    tile = tiles[0]

    # 3. Mover robot a baldosa elegida
    robot.moveToTile(tile)

    # print all tiles that are valids
    for t in robot.map.tiles.values():
        print(t.col,t.row, t.isValid())

    # 4. Repetir
    