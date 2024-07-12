from map import Tile, TileType, Map
from point import Point
from piso import Piso
from rectangle import Rectangle

import math

from dataclasses import dataclass

@dataclass(frozen=True)
class Movement:
    start : tuple[int, int] 
    dest : tuple[int, int]

class Navigator:

    def whereToGo(self, robot):
        return robot.position
    
class Navigator1(Navigator):

    def __init__(self):
        self.blockedPaths = set()

    def addBlockedPath(self, start, dest):
        start, dest = self.positionToGrid(start, dest)
        movement = Movement(start, dest)
        self.blockedPaths.add(movement)

    def positionToGrid(self, pos, origin):
        columna = round((pos.x - origin.x) / Tile.WIDTH)
        fila = round((pos.y - origin.y) / Tile.HEIGHT)
        return (columna, fila)

    def isObstructed(self, tile, robot):
        tile_pos = robot.map.gridToPosition(tile.col, tile.row)
        rect = robot.map.getTileRectangle(tile.col, tile.row)
        if tile.type == TileType.BLACK_HOLE:
            return True
        if not tile.isOpenAt(tile_pos):
            return True
        for obstacle in robot.map.obstacles:
            obstacle_rect = robot.map.getObstacleRectangle(obstacle)
            if obstacle_rect.intersects(rect):
                return True

        return False
    
    def removeObstructed(self, neighbours, robot):
        start = self.positionToGrid(robot.position, robot.posicion_inicial)
        result = []
        for tile in neighbours:
            movement = Movement(start, (tile.col, tile.row))
            if movement not in self.blockedPaths:
                if not self.isObstructed(tile, robot):
                    result.append(tile)
        return result

    def checkNeighbours(self, robot):
        orient = robot.obtener_orientacion(robot.rotation)
        col, row = self.positionToGrid(robot.position, robot.posicion_inicial)
        print(f'posicion actual: ({col}, {row})')
        current_tile = robot.map.getTileAt(col, row)
        tile_order = {"N": ((-1, 0), (0, -1), (1, 0), (0, 1)),
                      "E": ((0, -1), (1, 0), (0, 1), (-1, 0)),
                      "S": ((1, 0), (0, 1), (-1, 0), (0, -1)),
                      "W": ((0, 1), (-1, 0), (0, -1), (1, 0))}
        tiles = []
        for c, r in tile_order[orient]:
            tile = robot.map.getTileAt(col + c, row + r)
            if current_tile.isConnectedTo(tile) \
                and not tile.type == TileType.BLACK_HOLE \
                and not tile.hasObstacle():
                tiles.append(tile)
        return tiles

    def whereToGo(self, robot):
        tiles = self.checkNeighbours(robot)
        tiles = self.removeObstructed(tiles, robot)
        tiles.sort(key=lambda t: t.visits)
        tile = tiles[0]
        return robot.map.gridToPosition(tile.col, tile.row)
    
class Navigator2(Navigator):
    def __init__(self):
        self.minitiles = {} # (c,r) -> visits
        self.blockedPaths = set()

    def addBlockedPath(self, start, dest):
        start = self.positionToMiniGrid(start)
        dest = self.positionToMiniGrid(dest)
        movement = Movement(start, dest)
        self.blockedPaths.add(movement)

    def positionToMiniGrid(self, pos): 
        # pos ya tiene que venir relativa a la posición inicial del robot
        half_width = Tile.WIDTH/2
        half_height = Tile.HEIGHT/2

        columna = round(pos.x / half_width)
        fila = round(pos.y / half_height)
        return (columna, fila)

    def getNeighbours(self, coords):
        result = []
        c = coords[0]
        r = coords[1]
        for d in [(-1, 0), (-1, -1), (0, -1), (1, -1), \
                  ( 1, 0), ( 1,  1), (0,  1), (-1, 1)]:
            result.append((c + d[0], r + d[1]))
        return result
    
    def removeObstructed(self, neighbours, robot):
        start = self.positionToMiniGrid(robot.position.subtract(robot.posicion_inicial))
        result = []
        for minitile in neighbours:
            movement = Movement(start, minitile)
            if movement not in self.blockedPaths:
                if not self.isObstructed(minitile, robot):
                    result.append(minitile)
        return result
    
    def isObstructed(self, minitile, robot):
        minitile_pos = self.getPosition(minitile, robot)

        rect = self.getRectangle(minitile_pos)

        tiles = robot.map.getTilesIntersecting(rect)

        for tile in tiles:
            if not tile.isOpenAt(minitile_pos):
                return True
            if tile.type == TileType.BLACK_HOLE:
                return True
            
        for obstacle in robot.map.obstacles:
            obstacle_rect = robot.map.getObstacleRectangle(obstacle)
            if obstacle_rect.intersects(rect):
                return True
            
        return False
            
    def getPosition(self, minitile, robot):
        c = minitile[0]
        r = minitile[1]

        x = c * Tile.WIDTH/2 + robot.posicion_inicial.x
        y = r * Tile.HEIGHT/2 + robot.posicion_inicial.y
        return Point(x, y)

    def getRectangle(self, minitile_pos):        
        left = minitile_pos.x - Tile.WIDTH/4
        right = minitile_pos.x + Tile.WIDTH/4
        top = minitile_pos.y - Tile.HEIGHT/4
        bottom = minitile_pos.y + Tile.HEIGHT/4
        return Rectangle(top, left, bottom, right)

    def incrementVisits(self, minitile):
        # (c,r) -> visits 
        previous_value = self.minitiles.get(minitile, 0)
        self.minitiles[minitile] = previous_value + 1

    def shiftByRotation(self, neighbours, rotation):
        delta_angle = (math.pi*2)/8
        idx = int(round(rotation / -delta_angle)) % 8
        return neighbours[idx:] + neighbours[:idx]

    def whereToGo(self, robot):
        # 1) Encontrar la minitile en la que está el robot
        x = robot.position.x - robot.posicion_inicial.x
        y = robot.position.y - robot.posicion_inicial.y
        minitile_coord = self.positionToMiniGrid(Point(x, y))
        self.incrementVisits(minitile_coord)
        
        # 2) Calcular las minitiles vecinas
        neighbours = self.getNeighbours(minitile_coord)
        neighbours = self.shiftByRotation(neighbours, robot.rotation)

        # 3) Eliminar las minitiles que tienen paredes
        neighbours = self.removeObstructed(neighbours, robot)
        
        # 4) Elegimos una que tenga la menor cantidad de visitas
        # target = random.choice(neighbours) # TODO(Richo): No usar random!
        neighbours.sort(key=lambda minitile: self.minitiles.get(minitile, 0))
        target = neighbours[0]

        # 5) Devolvemos el punto central de esa minitile
        return self.getPosition(target, robot)