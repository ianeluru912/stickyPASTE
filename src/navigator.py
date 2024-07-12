from map import Tile, TileType
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

    def __init__(self, robot) -> None:
        self._robot = robot
        self.blockedPaths = set()

    def addBlockedPath(self, start, dest):
        raise NotImplementedError()

    def whereToGo(self):
        return self._robot.position
    
class Navigator1(Navigator):

    def addBlockedPath(self, start, dest):
        start_coords = self._robot.map.positionToGrid(start)
        dest_coords = self._robot.map.positionToGrid(dest)
        movement = Movement(start_coords, dest_coords)
        self.blockedPaths.add(movement)

    def whereToGo(self):
        tiles = self.checkNeighbours()
        tiles.sort(key=lambda t: t.visits)
        tile = tiles[0]
        return self._robot.map.gridToPosition(tile.col, tile.row)
    
    def checkNeighbours(self):
        robot = self._robot
        orient = robot.obtener_orientacion(robot.rotation)
        col, row = robot.map.positionToGrid(robot.position)
        current_tile = robot.map.getTileAt(col, row)
        tile_order = {"N": ((-1, 0), (0, -1), (1, 0), (0, 1)),
                      "E": ((0, -1), (1, 0), (0, 1), (-1, 0)),
                      "S": ((1, 0), (0, 1), (-1, 0), (0, -1)),
                      "W": ((0, 1), (-1, 0), (0, -1), (1, 0))}
        tiles = []
        for c, r in tile_order[orient]:
            tile = robot.map.getTileAt(col + c, row + r)
            movement = Movement((col, row), (tile.col, tile.row))
            if current_tile.isConnectedTo(tile) \
                and not tile.type == TileType.BLACK_HOLE \
                and movement not in self.blockedPaths:
                tiles.append(tile)
        return tiles
    
class Navigator2(Navigator):
    def __init__(self, robot):
        super().__init__(robot)
        self.minitiles = {} # (c,r) -> visits

    def addBlockedPath(self, start, dest):
        start_coords = self.positionToMiniGrid(start)
        dest_coords = self.positionToMiniGrid(dest)
        movement = Movement(start_coords, dest_coords)
        self.blockedPaths.add(movement)

    def positionToMiniGrid(self, pos): 
        half_width = Tile.WIDTH/2
        half_height = Tile.HEIGHT/2

        columna = round((pos.x - self._robot.posicion_inicial.x) / half_width)
        fila = round((pos.y - self._robot.posicion_inicial.y) / half_height)
        return (columna, fila)

    def getNeighbours(self, coords):
        result = []
        c = coords[0]
        r = coords[1]
        for d in [(-1, 0), (-1, -1), (0, -1), (1, -1), \
                  ( 1, 0), ( 1,  1), (0,  1), (-1, 1)]:
            result.append((c + d[0], r + d[1]))
        return result
    
    def removeObstructed(self, neighbours):
        start = self.positionToMiniGrid(self._robot.position)
        result = []
        for minitile in neighbours:
            movement = Movement(start, minitile)
            if movement not in self.blockedPaths:
                if not self.isObstructed(minitile):
                    result.append(minitile)
        return result
    
    def isObstructed(self, minitile):
        minitile_pos = self.getPosition(minitile)

        rect = self.getRectangle(minitile_pos)

        tiles = self._robot.map.getTilesIntersecting(rect)

        for tile in tiles:
            if not tile.isOpenAt(minitile_pos):
                return True
            if tile.type == TileType.BLACK_HOLE:
                return True
            
        for obstacle in self._robot.map.obstacles:
            obstacle_rect = self._robot.map.getObstacleRectangle(obstacle)
            if obstacle_rect.intersects(rect):
                return True
            
        return False
            
    def getPosition(self, minitile):
        c = minitile[0]
        r = minitile[1]

        x = c * Tile.WIDTH/2 + self._robot.posicion_inicial.x
        y = r * Tile.HEIGHT/2 + self._robot.posicion_inicial.y
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

    def whereToGo(self):
        minitile_coord = self.positionToMiniGrid(self._robot.position)
        self.incrementVisits(minitile_coord)
        
        # 2) Calcular las minitiles vecinas
        neighbours = self.getNeighbours(minitile_coord)
        neighbours = self.shiftByRotation(neighbours, self._robot.rotation)

        # 3) Eliminar las minitiles que tienen paredes
        neighbours = self.removeObstructed(neighbours)
        
        # 4) Elegimos una que tenga la menor cantidad de visitas
        # target = random.choice(neighbours) # TODO(Richo): No usar random!
        neighbours.sort(key=lambda minitile: self.minitiles.get(minitile, 0))
        target = neighbours[0]

        # 5) Devolvemos el punto central de esa minitile
        return self.getPosition(target)