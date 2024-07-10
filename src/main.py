from robot import Robot
from point import Point
import cv2
from math import pi as PI
from map import TileType
from piso import Piso

robot = Robot()

# robot.moveToPoint(Point(robot.position.x-0.28, robot.position.y))
# robot.moveToPoint(Point(robot.position.x, robot.position.y+0.02))
# b,g,r,_=robot.colorSensor.getImage()
# print(r,g,b)
# robot.parar()
# robot.delay(10000)

while robot.step() != -1:
    navigator = robot.getNavigator()
    point = navigator.whereToGo(robot)

    robot.moveToPoint(point)

    # for tile in robot.map.getValidTiles():
    #     print(tile)
    #     print("/////////////////////////////////////")
    #     print(tile.getRepresentation())
    #     print("-------------------------------------")

    # print("############################################################")
    # valid_tiles = robot.map.getValidTiles()
    # for tile in valid_tiles:
    #     print(tile.col, tile.row)
    #     print(tile.tokensWest)
    #     print(tile.tokensVerticalInternalWall)
    #     print(tile.tokensEast)
    #     print("------")
