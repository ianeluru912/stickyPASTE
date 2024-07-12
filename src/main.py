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
    point = navigator.whereToGo()

    robot.moveToPoint(point)

    # if robot.timeRemaining < 300 or robot.realTimeRemaining < 300:
    #     print(robot.map.getTileAt(0,1))
    #     print(robot.map.getTileAt(1,2))
    #     rep=robot.map.getRepresentation()
    #     robot.comm.sendMap(rep)
    #     # print(rep)
    #     robot.comm.sendExit()
    # print(robot.map.getRepresentation())
    # # for tiles in robot.map.getValidTiles():
    # #     print(tiles)
    # print("############################################################")
    # valid_tiles = robot.map.getValidTiles()
    # for tile in valid_tiles:
    #     print(tile.col, tile.row)
    #     print(tile.tokensWest)
    #     print(tile.tokensVerticalInternalWall)
    #     print(tile.tokensEast)
    #     print("------")
