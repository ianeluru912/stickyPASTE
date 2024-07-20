from robot import Robot
from math import pi as PI

robot = Robot()

me_aleje = False
while robot.step() != -1:
    navigator = robot.getNavigator()
    point, shouldBrake = navigator.whereToGo()
    print('haciendo primer while')
    if not robot.lidar.hayAlgoIzquierda():
        robot.girar(PI/2)
        robot.avanzar(0.12, True)
    elif not robot.lidar.hayAlgoAdelante():
        robot.avanzar(0.12, True)
    elif not robot.lidar.hayAlgoDerecha():
        robot.girar(-PI/2)
        robot.avanzar(0.12, True)
    else:
        robot.girar(PI)
        robot.avanzar(0.12, True)
    me_aleje = True
    if me_aleje and robot.map.positionToGrid(robot.position) == (0,0):
        robot.estoy_en_segunda_vuelta = True
        break


while robot.step() != -1:
    navigator = robot.getNavigator()
    point, shouldBrake = navigator.whereToGo()

    sendMapNow = robot.position.distance_to(point) < 0.025
    
    if not sendMapNow:
        robot.moveToPoint(point, shouldBrake)

    if robot.timeRemaining < 10 or robot.realTimeRemaining < 10 or sendMapNow:
        robot.comm.sendExit()
        print(f'I got milk {robot.h_counts} times ')
        print(f'I picked up bread {robot.s_counts} times')
        print(f'I took eggs {robot.f_counts} times')
        print(f'I picked up pinneaple juice {robot.o_counts} times')

# distances = {'N': robot.lidar.rangeImage[256], 'W': robot.lidar.rangeImage[128], 'S': robot.lidar.rangeImage[0], 'E': robot.lidar.rangeImage[384]}
# #obtener la key del mayor valor en distances
# max_dir = max(distances, key = distances.get)
# max_val = distances[max_dir]
# print(max_dir)
# if max_dir == 'N':
#     robot.girar_absoluto(0, 0.01)
# elif max_dir == 'W':
#     robot.girar_absoluto(PI/2, 0.01)
# elif max_dir == 'S':
#     robot.girar_absoluto(PI, 0.01)
# else:
#     robot.girar_absoluto(-PI/2, 0.01)
# robot.avanzar(max_val - 0.06, True)



    # # for tiles in robot.map.getValidTiles():
    #     print(tiles)
    # print("############################################################")
    # valid_tiles = robot.map.getValidTiles()
    # for tile in valid_tiles:
    #     print(tile.col, tile.row)
    #     print(tile.tokensWest)
    #     print(tile.tokensVerticalInternalWall)
    #     print(tile.tokensEast)
    #     print("------")
