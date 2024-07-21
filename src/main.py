from robot import Robot
from math import pi as PI

robot = Robot()

me_aleje = False
while robot.step() != -1:
    if not robot.lidar.hayAlgoIzquierda():
        left_tile = robot.get_tile_izq()
        robot.moveToPoint((robot.map.gridToPosition(left_tile.col, left_tile.row)), True)
    elif not robot.lidar.hayAlgoAdelante():
        next_tile = robot.get_tile_ahead()
        robot.moveToPoint((robot.map.gridToPosition(next_tile.col, next_tile.row)), True)
    elif not robot.lidar.hayAlgoDerecha():
        right_tile = robot.get_tile_der()
        robot.moveToPoint((robot.map.gridToPosition(right_tile.col, right_tile.row)), True)
    else:
        robot.girar(PI)
        next_tile = robot.get_tile_ahead()
        robot.moveToPoint((robot.map.gridToPosition(next_tile.col, next_tile.row)), True)
        
    me_aleje = True
    
    if me_aleje and robot.map.positionToGrid(robot.position) == (0,0):
        robot.estoy_en_segunda_vuelta = True
        break

while robot.step() != -1:
    navigator = robot.getNavigator()
    point, shouldBrake = navigator.whereToGo()

    sendMapNow = robot.position.distance_to(point) < 0.025
    
    if robot.estoy_en_segunda_vuelta == True:
        if robot.map.positionToGrid(point) not in robot.navigator.minitiles:
            robot.floating_tile = True
    
    robot.moveToPoint(point, True)
    robot.floating_tile = False

    if robot.timeRemaining < 10 or robot.realTimeRemaining < 10 or sendMapNow:
        robot.comm.sendExit()
        print(f'I got milk {robot.h_counts} times ')
        print(f'I picked up bread {robot.s_counts} times')
        print(f'I took eggs {robot.f_counts} times')
        print(f'I picked up pinneaple juice {robot.o_counts} times')