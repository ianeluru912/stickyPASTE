from map import Tile, TileType
from point import Point
from piso import Piso
from rectangle import Rectangle
import utils

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

    def getNeighbours1(self, coords):
        result = []
        c = coords[0]
        r = coords[1]
        for d in [(-2, 0), (0, -2), (2, 0), (0, 2)]:
            result.append((c + d[0], r + d[1]))
        return result
    
    def getNeighbours2(self, coords):
        result = []
        c = coords[0]
        r = coords[1]
        for d in [(-1, 0), (-1, -1), (0, -1), (1, -1), \
                  ( 1, 0), ( 1,  1), (0,  1), (-1, 1)]:
            result.append((c + d[0], r + d[1]))
        return result
    
    def removeObstructed1(self, neighbours):
        robot = self._robot
        if self.getRotationIdx(robot.rotation, 8) % 2 != 0: 
            return []
        
        valid_x = utils.near_multiple(robot.position.x - robot.posicion_inicial.x, 0.12, 0.0025)
        valid_y = utils.near_multiple(robot.position.y - robot.posicion_inicial.y, 0.12, 0.0025)
        if not (valid_x and valid_y):
            return []
        
        start = self.positionToMiniGrid(robot.position)
        col, row = robot.map.positionToGrid(robot.position)
        current_tile = robot.map.getTileAt(col, row)
        tiles = []
        for minitile in neighbours:
            pos = self.getPosition(minitile)
            tile = robot.map.getTileAtPosition(pos)
            movement = Movement(start, minitile)
            inbetween = (minitile[0] + start[0]) / 2, (minitile[1] + start[1]) / 2
            if current_tile.isConnectedTo(tile) \
                    and not tile.type == TileType.BLACK_HOLE \
                    and movement not in self.blockedPaths \
                    and not self.isObstructed(minitile) \
                    and not self.isObstructed(inbetween):
                tiles.append(minitile)
        return tiles
    
    def removeObstructed2(self, neighbours):
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

    def getRotationIdx(self, rotation, count):
        delta_angle = (math.pi*2)/count
        return int(round(rotation / -delta_angle)) % count

    def shiftByRotation(self, neighbours, rotation):
        rotation_idx = self.getRotationIdx(rotation, len(neighbours))
        return neighbours[rotation_idx:] + neighbours[:rotation_idx]

    def whereToGo(self):
        minitile_coord = self.positionToMiniGrid(self._robot.position)
        self.incrementVisits(minitile_coord)

        self._robot.mapvis.send_minitiles(self._robot)
        
        # 2) Calcular las minitiles vecinas
        neighbours1 = self.getNeighbours1(minitile_coord)
        neighbours1 = self.shiftByRotation(neighbours1, self._robot.rotation)
        neighbours2 = self.getNeighbours2(minitile_coord)
        neighbours2 = self.shiftByRotation(neighbours2, self._robot.rotation)

        # 3) Eliminar las minitiles que tienen paredes
        neighbours1 = self.removeObstructed1(neighbours1)
        neighbours2 = self.removeObstructed2(neighbours2)
                
        # 4) Elegimos una que tenga la menor cantidad de visitas
        currentTile = self._robot.map.getTileAtPosition(self._robot.position)
        if not currentTile.isColorPassage():
            neighbours1.sort(key=lambda minitile: self.minitiles.get(minitile, 0))
            neighbours2.sort(key=lambda minitile: self.minitiles.get(minitile, 0))
        target1 = neighbours1[0] if len(neighbours1) > 0 else None
        target2 = neighbours2[0] if len(neighbours2) > 0 else None
        print(f"T1: {target1}, T2: {target2}")

        target = None
        if target1 == None:
            target = target2
            print("TARGET1 es None")
        elif self.minitiles.get(target1, 0) <= self.minitiles.get(target2, 0):
            target = target1
            print("TARGET1 tiene menos visitas")
        else:
            target = target2
            print("TARGET1 tiene más visitas")

        # 5) Devolvemos el punto central de esa minitile
        return self.getPosition(target)