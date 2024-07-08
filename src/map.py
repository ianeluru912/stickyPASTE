from point import Point
from rectangle import Rectangle
from enum import Enum
import numpy as np

class TileType(Enum):
    BLACK_HOLE = '2'
    SWAMP = '3'
    CHECKPOINT = '4'
    STARTING = '5'
    BLUE = 'b'
    YELLOW = 'y'
    GREEN = 'g'
    PURPLE = 'p'
    ORANGE = 'o'
    RED = 'r'
    STANDARD = 's'

class Map:
    def __init__(self, origin) -> None:
        self.origin = origin
        self.tiles = {}

    def gridToPosition(self, col, row):
        x = self.origin.x + col * Tile.WIDTH
        y = self.origin.y + row * Tile.HEIGHT
        return Point(x, y)

    def positionToGrid(self, pos):
        columna = round((pos.x - self.origin.x) / Tile.WIDTH)
        fila = round((pos.y - self.origin.y) / Tile.HEIGHT)
        return (columna, fila)

    def getTileAt(self, col, row):
        tile = self.tiles.get((col, row))
        if tile == None:
            tile = Tile(col, row, self)
            self.tiles[(col, row)] = tile
        return tile
    
    def getTileAtPosition(self, point):
        col, row = self.positionToGrid(point)
        return self.getTileAt(col, row)
    
    def getTileRectangle(self, col, row):
        position = self.gridToPosition(col, row)
        left = position.x - Tile.WIDTH/2
        right = position.x + Tile.WIDTH/2
        top = position.y - Tile.HEIGHT/2
        bottom = position.y + Tile.HEIGHT/2
        return Rectangle(top, left, bottom, right)
    
    def getTilesIntersecting(self, rectangle):
        result = []
        for tile in self.tiles.values():
            tile_rect = self.getTileRectangle(tile.col, tile.row)
            if tile_rect.intersects(rectangle):
                result.append(tile)
        return result
            
    def writeMap(self, file_path, robot):
        col, row = self.positionToGrid(robot.position)
        orientation = robot.obtener_orientacion(robot.rotation)
        # Primero buscamos los extremos del mapa, que nos servira para iterar
        # en la grilla en el orden correcto.
        min_col, min_row, max_col, max_row = 0, 0, 0, 0
        for c, r in self.tiles:
            t = self.tiles.get((c, r))
            if t == None or not t.isConnected(): continue # Ignoramos baldosas aisladas
            if c < min_col: min_col = c
            if r < min_row: min_row = r
            if c > max_col: max_col = c
            if r > max_row: max_row = r
        # Luego revisamos una por una cada baldosa y calculamos los caracteres
        # a imprimir de acuerdo a sus caracteristicas (si es la baldosa de
        # inicio, si tiene vecinos cuales, si fue visitada, etc.)
        char_groups = []
        for r in range(min_row, max_row + 1):
            char_group = []
            for c in range(min_col, max_col + 1):
                t = self.tiles.get((c, r))
                chars = [["+", "-", "+"],
                        ["|", "X", "|"],
                        ["+", "-", "+"]]
                if t != None:
                    if c == col and r == row:
                        chars[1][1] = orientation
                    elif c == 0 and r == 0:
                        chars[1][1] = "@"
                    elif not t.isConnected():
                        chars[1][1] = "X"
                    elif t.type==TileType.BLACK_HOLE:
                        chars[1][1] = "B"
                    elif t.type==TileType.BLUE:
                        chars[1][1] = "A"
                    elif t.visits == 0:
                        chars[1][1] = "?"
                    # elif t.hasObstacle:
                    #     chars[1][1] = "O"
                    else:
                        chars[1][1] = " "
                    
                    if t.north != None: chars[0][1] = " "
                    if t.east != None: chars[1][2] = " "
                    if t.south != None: chars[2][1] = " "
                    if t.west != None: chars[1][0] = " "
                char_group.append(chars)
            char_groups.append(char_group)
        # Finalmente, abrimos el archivo y escribimos los caracteres que
        # representan a cada baldosa
        with open(file_path, "w") as f:
            for char_group in char_groups:
                for i in range(0, 3):
                    for chars in char_group:
                        f.write("".join(chars[i]))
                    f.write("\n")
    
    def getValidTiles(self):
        salida=[tile for tile in self.tiles.values() if tile.isValid()]
        return salida
    
class Tile:
    WIDTH = 0.12  
    HEIGHT = 0.12

    def __init__(self, col, row, map) -> None:
        self.__map = map
        self.col = col
        self.row = row
        self.visits = 0
        
        self.north = [-1, -1, -1]
        self.west = [-1, -1, -1]
        self.east = [-1, -1, -1]
        self.south = [-1, -1, -1]

        self.tokensNorth = [0,0,0]
        self.tokensWest = [0,0,0]
        self.tokensEast = [0,0,0]
        self.tokensSouth = [0,0,0]

        self.tokensVerticalInternalWall=[0,0]
        self.tokensHorizontalInternalWall=[0,0]

        self.type=None
        self.hasObstacle = False
        self.area = None
    
    def __str__(self) -> str:
        return f"Tile ({self.col}, {self.row}) tipo: {self.type} visitas: {self.visits}	area: {self.area}"
    
    def getRepresentation(self):
        # create a numpy array 5x5
        # Agregar en la tile las víctimas y carteles ACAACA
        # Agregar paredes internas
        # * * * * * 
        # * * * * * 
        # * * * * * 
        # * * * * * 
        # * * * * * 
        

        rep=np.full((5,5), None)
        # Agregar las paredes externas
        rep[0,0:5]=self.combinesWall(rep[0,0:5], self.getWallRepresentation(self.north))
        rep[4,0:5]=self.combinesWall(rep[4,0:5], self.getWallRepresentation(self.south))
        rep[0:5,0]=self.combinesWall(rep[0:5,0],self.getWallRepresentation(self.west))
        rep[0:5,4]=self.combinesWall(rep[0:5,4],self.getWallRepresentation(self.east))

        # TODO:Agregar las paredes internas

        # Agregar el color de la baldosa
        if self.type==TileType.GREEN:
            rep[1, 1]="g"
            rep[1, 3]="g"
            rep[3, 1]="g"
            rep[3, 3]="g"
        elif self.type==TileType.BLUE:
            rep[1, 1]="b"
            rep[1, 3]="b"
            rep[3, 1]="b"
            rep[3, 3]="b"
        elif self.type==TileType.YELLOW:
            rep[1, 1]="y"
            rep[1, 3]="y"
            rep[3, 1]="y"
            rep[3, 3]="y"
        elif self.type==TileType.PURPLE:
            rep[1, 1]="p"
            rep[1, 3]="p"
            rep[3, 1]="p"
            rep[3, 3]="p"
        elif self.type==TileType.ORANGE:
            rep[1, 1]="o"
            rep[1, 3]="o"
            rep[3, 1]="o"
            rep[3, 3]="o"
        elif self.type==TileType.RED:
            rep[1, 1]="r"
            rep[1, 3]="r"
            rep[3, 1]="r"
            rep[3, 3]="r"
        elif self.type==TileType.BLACK_HOLE:
            rep[1, 1]="2"
            rep[1, 3]="2"
            rep[3, 1]="2"
            rep[3, 3]="2"
        elif self.type==TileType.SWAMP:
            rep[1, 1]="3"
            rep[1, 3]="3"
            rep[3, 1]="3"
            rep[3, 3]="3"
        elif self.type==TileType.CHECKPOINT:
            rep[1, 1]="4"
            rep[1, 3]="4"
            rep[3, 1]="4"
            rep[3, 3]="4"
        elif self.type==TileType.STARTING:
            rep[1, 1]="5"
            rep[1, 3]="5"
            rep[3, 1]="5"
            rep[3, 3]="5"
        # TODO: Agregar las víctimas y carteles

        for fil in range(rep.shape[0]):
                for colum in range(rep.shape[1]):
                    if rep[fil, colum] is None:
                        rep[fil, colum] = '0'
        return rep

    def maxWall(self, v1, v2):
        if v1==1 or v2==1:
            return 1
        elif v1==0 or v2==0:
            return 0
        else:
            return None

    def combinesWall(self, w1, w2):
        size=len(w1)
        sol=np.full(size, None)
        for i in range(size):
            sol[i]=self.maxWall(w1[i],w2[i])
        return sol

    def getWallRepresentation(self, wall):
        left=wall[0]
        right=wall[2]
        if left==-1 and right==-1:
            return [None, None, None, None, None]
        elif left==-1 and right==0:
            return [None, None, 0, 0, 0]
        elif left==-1 and right==1:
            return [None, None, 1, 1, 1]
        elif left==0 and right==-1:
            return [0, 0, 0, None, None]
        elif left==0 and right==0:
            return [0, 0, 0, 0, 0]
        elif left==0 and right==1:
            return [0, 0, 1, 1, 1]
        elif left==1 and right==-1:
            return [1, 1, 1, None, None]
        elif left==1 and right==0:
            return [1, 1, 1, 0, 0]
        elif left==1 and right==1:
            return [1, 1, 1, 1, 1]


    def getDirectionTo(self, tile):
        sc = self.col
        sr = self.row
        dc = tile.col
        dr = tile.row
        if dc - sc == 0: # Misma columna
            if dr - sr > 0: return "S"
            if dr - sr < 0: return "N"
        elif dr - sr == 0: # Misma fila
            if dc - sc > 0: return "E"
            if dc - sc < 0: return "W"
        return None
    
    def isConnectedTo(self, tile):
        direction = self.getDirectionTo(tile)
        if direction == "N":
            return self.north == [0, 0, 0]
        elif direction == "S":
            return self.south == [0, 0, 0]
        elif direction == "E":
            return self.east == [0, 0, 0]
        elif direction == "W":
            return self.west == [0, 0, 0]
        return False
    
    def isConnected(self):
        if self.north != None: 
            return True
        elif self.west != None:
            return True
        elif self.east != None:
            return True
        elif self.south != None:
            return True
        return False
    
    def set_area(self, area):
        self.area = area

    def get_area(self):
        return self.area
    
    # def get_color(self):
    #     if self.isBlue:
    #         return 'Blue'
    #     elif self.isPurple:
    #         return 'Purple'
    #     elif self.isYellow:
    #         return 'Yellow'
    #     elif self.isGreen:
    #         return 'Green'
    #     elif self.isRed:
    #         return 'Red'
    #     elif self.isOrange:
    #         return 'Orange'
    #     else:
    #         return None
    
    def isValid(self):
        #join self.north,self.west,self.east,self.south
        walls=self.north+self.west+self.east+self.south
        # if there is a 0 in the walls, then it is a valid tile
        return 0 in walls
    
    def isOpenAt(self, pos):
        center = self.__map.gridToPosition(self.col, self.row)
        thresh = 0.02
        if pos.x < center.x - thresh:
            if pos.y < center.y - thresh:
                # NW
                return self.north[0] <= 0 and self.west[2] <= 0
            elif pos.y > center.y + thresh:
                # SW
                return self.south[2] <= 0 and self.west[0] <= 0
            else:
                # W
                return self.west[0] <= 0 and self.west[1] <= 0 and self.west[2] <= 0
        elif pos.x > center.x + thresh:
            if pos.y < center.y - thresh:
                # NE
                return self.north[2] <= 0 and self.east[0] <= 0
            elif pos.y > center.y + thresh:
                # SE
                return self.east[2] <= 0 and self.south[0] <= 0
            else:
                # E
                return self.east[0] <= 0 and self.east[1] <= 0 and self.east[2] <= 0
        else:
            if pos.y < center.y - thresh:
                # N
                return self.north[0] <= 0 and self.north[1] <= 0 and self.north[2] <= 0
            elif pos.y > center.y + thresh:
                # S
                return self.south[0] <= 0 and self.south[1] <= 0 and self.south[2] <= 0
            else: 
                # CENTRO!
                return self.north[1] <= 0 and self.east[1] <= 0 and self.south[1] <= 0 and self.west[1] <= 0
