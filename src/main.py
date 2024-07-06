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


# robot.moveToPoint(Point(robot.position.x, robot.position.y+0.18))
# robot.moveToPoint(Point(robot.position.x+0.36, robot.position.y))
# robot.parar()
# valid_tiles = robot.map.getValidTiles()
# for tile in valid_tiles:
#     print(tile.col, tile.row)
#     print(tile.tokensNorth)
#     print(tile.tokensHorizontalInternalWall)
#     print(tile.tokensSouth)
#     print("------")


while robot.step() != -1:
    print("Loop")
    robot.updateMap() 

    navigator = robot.getNavigator()
    point = navigator.whereToGo(robot)

    robot.moveToPoint(point)
 
#     valid_tiles = robot.map.getValidTiles()
#     for tile in valid_tiles:
#         print(tile.col, tile.row)
#         print(tile.tokensWest)
#         print(tile.tokensVerticalInternalWall)
#         print(tile.tokensEast)
#         print("------")
