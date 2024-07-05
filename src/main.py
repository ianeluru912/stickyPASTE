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
    robot.updateMap() 

    """navigator = robot.getNavigator()
    point = navigator.whereToGo(robot)

    robot.moveToPoint(point)"""

   # valid_tiles = robot.map.getValidTiles()
    # for tile in valid_tiles:
    #     print(tile)
    #     print(tile.getRepresentation())