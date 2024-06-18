from robot import Robot
from point import Point
robot = Robot()

while robot.step() != -1:
    # 0. Actualizar mapa
    robot.updateMap()
    
    # 1. Dame baldosas vecinas candidatas a donde puedo moverme
    tiles = robot.checkNeighbours()
    
    # 2. Elegir una baldosa de las candidatas
    tiles.sort(key=lambda t: t.visits)
    tile = tiles[0]

    # 3. Mover robot a baldosa elegida
    robot.moveTo(tile)
    
    # 4. Repetir


