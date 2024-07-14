from map import Tile, TileType
from point import Point
from rectangle import Rectangle
from collections import deque 
from dataclasses import dataclass
import time

@dataclass(frozen=True)
class Movement:
    start : tuple[int, int] 
    dest : tuple[int, int]

class Navigator:
    def __init__(self, robot) -> None:
        self._robot = robot
        self.blockedPaths = set()
        self.minitiles = {} # (c,r) -> visits
        self.obstructed = set()

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

    def isObstructed(self, minitile):
        if minitile in self.minitiles: 
            return False
        if minitile in self.obstructed:
            return True
        
        minitile_pos = self.getPosition(minitile)
        rect = self.getRectangle(minitile_pos)
        tiles = self._robot.map.getTilesIntersecting(rect)

        for tile in tiles:
            if tile.dangerous or tile.type == TileType.BLACK_HOLE or not tile.isOpenAt(minitile_pos):
                self.obstructed.add(minitile)
                return True
            
        for obstacle in self._robot.map.obstacles:
            obstacle_rect = self._robot.map.getObstacleRectangle(obstacle)
            if obstacle_rect.intersects(rect):
                self.obstructed.add(minitile)
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

    def getNeighbours(self, coords):
        result = []
        c = coords[0]
        r = coords[1]
        for d in [(-1, 0), (0, -1), ( 1, 0), (0,  1), \
                  (-1, -1), (1, -1), ( 1,  1), (-1, 1)]:
            result.append((c + d[0], r + d[1]))
        return result
    
    def removeObstructed(self, start, neighbours):
        result = []
        for minitile in neighbours:
            movement = Movement(start, minitile)
            if movement not in self.blockedPaths:
                if not self.isObstructed(minitile):
                    result.append(minitile)
        return result
    
    def findPath(self):
        frontier = deque()

        start = self.positionToMiniGrid(self._robot.position)
        frontier.append(start)

        came_from = {} # path A->B is stored as came_from[B] == A
        came_from[start] = None

        target = None
        while len(frontier) > 0:
            current = frontier.popleft()

            if current != start and self.minitiles.get(current, 0) == 0:
                target = current
                break

            neighbours = self.getNeighbours(current)
            neighbours = self.removeObstructed(current, neighbours)
            for next in neighbours:
                if next not in came_from:
                    frontier.append(next)
                    came_from[next] = current


        # print(f"Buscando camino desde: {start}")

        if target is None:
            # print("Exploramos todo el mapa!!")
            # print("Volvamos al inicio...")
            target = (0, 0)

        # En came_from tenemos el camino desde el robot hasta cualquier baldosa
        # (incluida target)
        current = target 
        path = []
        while current is not None: 
            path.append(current)
            current = came_from.get(current)
        path.reverse() # optional

        # print(f"Target: {target}")
        # print(path)
        
        return path

    def whereToGo(self):
        minitile_coord = self.positionToMiniGrid(self._robot.position)
        self.incrementVisits(minitile_coord)

        self._robot.mapvis.send_minitiles(self._robot)
        
        # print("-------")
        begin_time = time.time()
        path = self.findPath()
        end_time = time.time()
        # print(f"Elapsed time: {(end_time - begin_time) * 1000} ms")
        target = path[1] if len(path) > 1 else path[0]
        # print(f"Actual target: {target}")

        shouldBrake = True
        if len(path) > 2:
            delta1 = (path[1][0] - path[0][0], path[1][1] - path[0][1])
            # print(f"D1: {delta1}")
            delta2 = (path[2][0] - path[1][0], path[2][1] - path[1][1])
            # print(f"D2: {delta2}")
            if delta1 == delta2:
                # print("PODEMOS NO FRENAR")
                shouldBrake = False
            
        return self.getPosition(target), shouldBrake