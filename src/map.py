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
        self.obstacles = []

    def addObstacle(self, point):
        self.obstacles.append(point)

    def getObstacleRectangle(self, obstacle):
        top = obstacle.y - 0.02
        left = obstacle.x - 0.02
        bottom = obstacle.y + 0.02
        right = obstacle.x + 0.02
        return Rectangle(top, left, bottom, right)

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
    
    def combineTilesReg(self, tileMapa, tileAUbicar):
        # en cada casilla del mapa que recibe tengo 1, 0, una letra o un *
        # None<*<0,1<letra
        for i in range(5):
            for j in range(5):
                # print(i,j)
                # Si lo que viene es un None, no lo pongo.
                if tileAUbicar[i, j] == None:
                    # print("Caso 1")
                    # No hace nada
                    continue

                # Sino, si lo que recibe es un None, pongo lo que venga
                elif tileMapa[i, j] == None:
                    # print("Caso 2")
                    tileMapa[i, j] = tileAUbicar[i, j]
 
                # Sino, si lo que tenía es un * pongo lo que venga (porque sé que no es un None)
                elif tileMapa[i, j] == "*":
                    # print("Caso 3")
                    tileMapa[i, j] = tileAUbicar[i, j]
                
                # Sino, si lo que tenía es un 0 o un 1, y lo que viene es una letra, pongo la letra
                elif (tileMapa[i, j] == "0" or tileMapa[i, j] == "1") and (tileAUbicar[i, j] != "0" and \
                    tileAUbicar[i, j] != "1" and tileAUbicar[i, j] != "*"):
                    # print("Caso 4")
                    tileMapa[i, j] = tileAUbicar[i, j]

                

    def combineTilesReal(self, tileMapa, tileAUbicar):
        # en cada casilla del mapa que recibe tengo 1, 0, una letra o un *
        # None<*<0,1<letra
        for i in range(5):
            for j in range(5):
                # print(i,j)
                # Si lo que viene es un None O LO QUE TENGO ES UN ASTERISCO, no lo pongo.
                if tileAUbicar[i, j] == None or tileMapa[i, j] == "*":
                    # print("Caso 1")
                    # No hace nada
                    continue

                # Sino, si lo que recibe es un None, pongo lo que venga
                elif tileMapa[i, j] == None or tileAUbicar[i, j] == "*":
                    # print("Caso 2")
                    tileMapa[i, j] = tileAUbicar[i, j]
 
                # Sino, si lo que tenía es un 0 o un 1, y lo que viene es una letra, pongo la letra
                elif (tileMapa[i, j] == "0" or tileMapa[i, j] == "1") and (tileAUbicar[i, j] != "0" and \
                    tileAUbicar[i, j] != "1"):
                    # print("Caso 4")
                    tileMapa[i, j] = tileAUbicar[i, j]
                
                elif tileMapa[i, j] != "1" and tileMapa[i, j] != "0" and tileAUbicar[i, j] != "1" and tileAUbicar[i, j] != "1":
                    old = tileMapa[i, j]
                    new = tileAUbicar[i, j]
                    if new in "HSU" and not (old in "HSU"):
                        tileMapa[i, j] = new + old
                    else:
                        tileMapa[i, j] = old + new

                # Si teníamos un 0 o un 1, y viene un 0 o un 1, debería ser coherente. No hacemos nada
    
    def getRepresentation(self):
        # obtener tiles validos
        valid_tiles = self.getValidTiles()
        #TODO: En los tiles de área 4, poner asteriscos (ver dónde)
        # detectar la columnna mínima y máxima de los tiles válidos
        colmin = min([tile.col for tile in valid_tiles])
        colmax = max([tile.col for tile in valid_tiles])
        rowmin = min([tile.row for tile in valid_tiles])
        rowmax = max([tile.row for tile in valid_tiles])

        totCol=(colmax-colmin+1)*4+1
        totRow=(rowmax-rowmin+1)*4+1
        repre=np.full((totRow, totCol), None)

        valid_tiles.sort(key=lambda t: t.col)
        valid_tiles.sort(key=lambda t: t.row)

        for tile in valid_tiles:

            # Calcula la columna donde tiene que poner el tile dentro de la representación
            col=(tile.col-colmin)*4
            # Calcula la fila donde tiene que poner el tile dentro de la representación
            row=(tile.row-rowmin)*4
            # Obtiene la representación del tile
            rep=tile.getRepresentation()
            # Combinar lo que ya había en la representación con lo que acabo de obtener.
            # if tile.col==1 and tile.row==1:
            #     print(f"Col: {tile.col}, Row: {tile.row}")
            #     print("Tile")
            #     print(rep)
            #     print("El que recibe el tile")
            #     print(repre[row:row+5, col:col+5])
            #     print("====")

            # ACAACA CAMBIAR POR COMBINETILESREG SI EL ASTERISCO ES LO QUE MENOS PESA (COMO MUESTRA EL EJEMPLO DEL REGLAMENTO)
            self.combineTilesReal(repre[row:row+5, col:col+5], rep)
            # if tile.col==1 and tile.row==1:
            #     print("Como quedó después de la combineta")
            #     print(repre[row:row+5, col:col+5])
        
        for i in range(totRow):
            for j in range(totCol):
                if repre[i, j] == None:
                    repre[i, j] = '0' #ACAACA Conviene poner 0 o *?????
        return repre

    def existTileAt(self, col, row):
        return (col, row) in self.tiles      

  
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
        self.area = None
        self.dangerous=False
    
    def __str__(self) -> str:
        return f"Tile ({self.col}, {self.row}) {self.getRepresentation()}"
        
    def getRectangle(self):
        return self.__map.getTileRectangle(self.col, self.row)

    def getNorthTile(self):
        return self.__map.getTileAt(self.col, self.row - 1)
    
    def getEastTile(self):
        return self.__map.getTileAt(self.col + 1, self.row)

    def getSouthTile(self):
        return self.__map.getTileAt(self.col, self.row + 1)

    def getWestTile(self):
        return self.__map.getTileAt(self.col - 1, self.row)
    
    def isNVOrA4OrAN(self):
        return not(self.isValid()) or self.area==4 or self.area==None
    
    def oneOrMoreNeighborsAreArea4(self):
        return self.getNorthTile().area==4 or self.getEastTile().area==4 or self.getSouthTile().area==4 or self.getWestTile().area==4

    def getRepresentation(self):
        # create a numpy array 5x5
        # Agregar en la tile las víctimas y carteles 
        # Agregar paredes internas
  
        # si es area 4, hacer todos * y retornarlo
        if self.area == 4:
            return np.full((5,5), "*")
        
        if self.area == None and self.oneOrMoreNeighborsAreArea4() and self.getNorthTile().isNVOrA4OrAN() and self.getEastTile().isNVOrA4OrAN() and self.getSouthTile().isNVOrA4OrAN() and self.getWestTile().isNVOrA4OrAN():
            return np.full((5,5), "*")
        

        rep=np.full((5,5), None)
        # Agregar las paredes externas
        # Dado que la representación de la pared sur y la oeste están invertidas, nos creamos una versión dada vuelta para generar
        # la representación de la pared sur y la oeste
        # create a copy of the south and west walls reversed
        south_reversed = self.south.copy()
        south_reversed.reverse()
        west_reversed = self.west.copy()
        west_reversed.reverse()

        # if self.col==0 and self.row==-1:
        #     print("Estoy analizando la baldosa 0,-1")
        #     print(self.__map.existTileAt(self.col-1, self.row))
        #     tile=self.__map.getTileAt(self.col-1, self.row)
        #     print(tile.isValid())

        if self.__map.getTileAt(self.col, self.row-1).isValid():
            northWall=self.getWallRepresentation(self.north)
        else:
            northWall=['1','1','1','1','1']

        if self.__map.getTileAt(self.col+1, self.row).isValid():
            eastWall=self.getWallRepresentation(self.east)
        else:
            eastWall=['1','1','1','1','1']

        if self.__map.getTileAt(self.col, self.row+1).isValid():
            southWall=self.getWallRepresentation(south_reversed)
        else:
            southWall=['1','1','1','1','1']

        if self.__map.getTileAt(self.col-1, self.row).isValid():
            westWall=self.getWallRepresentation(west_reversed)
        else:
            westWall=['1','1','1','1','1']


        # if not(self.__map.existTileAt(self.col, self.row+1)):
        #     southWall=['1','1','1','1','1']
        # else:
        #     southWall=self.getWallRepresentation(south_reversed)

        # if not(self.__map.existTileAt(self.col-1, self.row)):
        #     westWall=['1','1','1','1','1']
        # else:
        #     westWall=self.getWallRepresentation(west_reversed)

        rep[0,0:5]=self.combinesWall(rep[0,0:5], northWall)
        rep[0:5,4]=self.combinesWall(rep[0:5,4], eastWall)
        rep[4,0:5]=self.combinesWall(rep[4,0:5], southWall)
        rep[0:5,0]=self.combinesWall(rep[0:5,0], westWall)


        # TODO:Agregar las paredes internas
        # Pared interna vertical superior, es el valor del medio de self.north
        if self.north[1]==1:
            rep[0:3,2]=self.combinesWall(rep[0:3,2], ['1', '1', '1'])
        elif self.north[1]==0:
            rep[0:3,2]=self.combinesWall(rep[0:3,2], ['0', '0', '0'])
        # Pared interna vertical inferior, es el valor del medio de self.south
        if self.south[1]==1:
            rep[2:5,2]=self.combinesWall(rep[2:5,2], ['1', '1', '1'])
        elif self.south[1]==0:
            rep[2:5,2]=self.combinesWall(rep[2:5,2], ['0', '0', '0'])
        # Pared interna horizontal izquierda, es el valor del medio de self.west
        if self.west[1]==1:
            rep[2,0:3]=self.combinesWall(rep[2,0:3], ['1', '1', '1'])
        elif self.west[1]==0:
            rep[2,0:3]=self.combinesWall(rep[2,0:3], ['0', '0', '0'])
        # Pared interna horizontal derecha, es el valor del medio de self.east
        if self.east[1]==1:
            rep[2,2:5]=self.combinesWall(rep[2,2:5], ['1', '1', '1'])
        elif self.east[1]==0:
            rep[2,2:5]=self.combinesWall(rep[2,2:5], ['0', '0', '0'])


        # Agregar el color de la baldosa
        if not(self.type==TileType.STANDARD or self.type==None):
            rep[1, 1]=rep[1, 3]=rep[3, 1]=rep[3, 3]=self.type.value

        # DONE: Agregar las víctimas y carteles
        if self.tokensNorth != [0, 0, 0]:
            if self.tokensNorth[0] != 0:
                rep[0, 1] = self.tokensNorth[0]
            if self.tokensNorth[1] != 0:
                rep[0, 2] = self.tokensNorth[1]
            if self.tokensNorth[2] != 0:
                rep[0, 3] = self.tokensNorth[2]

        if self.tokensSouth != [0, 0, 0]:
            if self.tokensSouth[0] != 0:
                rep[4, 1] = self.tokensSouth[0]
            if self.tokensSouth[1] != 0:
                rep[4, 2] = self.tokensSouth[1]
            if self.tokensSouth[2] != 0:
                rep[4, 3] = self.tokensSouth[2]

        if self.tokensWest != [0, 0, 0]:
            if self.tokensWest[0] != 0:
                rep[1, 0] = self.tokensWest[0]
            if self.tokensWest[1] != 0:
                rep[2, 0] = self.tokensWest[1]
            if self.tokensWest[2] != 0:
                rep[3, 0] = self.tokensWest[2]

        if self.tokensEast != [0, 0, 0]:
            if self.tokensEast[0] != 0:
                rep[1, 4] = self.tokensEast[0]
            if self.tokensEast[1] != 0:
                rep[2, 4] = self.tokensEast[1]
            if self.tokensEast[2] != 0:
                rep[3, 4] = self.tokensEast[2]
        
        #TODO: Agregar vícitmas y carteles internos
        if not(self.tokensVerticalInternalWall[0]==0):
            rep[1,2]=self.tokensVerticalInternalWall[0]
        if not(self.tokensVerticalInternalWall[1]==0):
            rep[3,2]=self.tokensVerticalInternalWall[1]
        if not(self.tokensHorizontalInternalWall[0]==0):
            rep[2,1]=self.tokensHorizontalInternalWall[0]
        if not(self.tokensHorizontalInternalWall[1]==0):
            rep[2,3]=self.tokensHorizontalInternalWall[1]
        
        #Si es un black hole no me importa si detectó paredes internas o lo que sea, el formato es siempre el mismo.
        if self.type==TileType.BLACK_HOLE:
            rep[1,1:4]=['2','0','2']
            rep[2,1:4]=['0','0','0']
            rep[3,1:4]=['2','0','2']	
        
        for fil in range(1,rep.shape[0]-1):
                for colum in range(1,rep.shape[1]-1):
                    if rep[fil, colum] is None:
                        rep[fil, colum] = '0'
        
            
        return rep

    def maxWall(self, v1, v2):
        if v1=='1' or v2=='1':
            return '1'
        if v1=='0' and v2=='0':
            return '0'
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
            return [None, None, '0', '0', '0']
        elif left==-1 and right==1:
            return [None, None, '1', '1', '1']
        elif left==0 and right==-1:
            return ['0', '0', '0', None, None]
        elif left==0 and right==0:
            return ['0', '0', '0', '0', '0']
        elif left==0 and right==1:
            return ['0', '0', '1', '1', '1']
        elif left==1 and right==-1:
            return ['1', '1', '1', None, None]
        elif left==1 and right==0:
            return ['1', '1', '1', '0', '0']
        elif left==1 and right==1:
            return ['1', '1', '1', '1', '1']


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
        if self.isColorPassage(): return
        self.area = area

    def get_area(self):
        return self.area
    
    def isColorPassage(self):
        colored_types = [TileType.BLUE, TileType.YELLOW, TileType.GREEN, \
                         TileType.PURPLE, TileType.ORANGE, TileType.RED]
        return self.type in colored_types
    
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

    def setTokenOnAWall(self, point, token):
        # print(token)
        # print(self.col, self.row)
        center = self.__map.gridToPosition(self.col, self.row)
        umbralChico = 0.015 #0.02
        umbralGrande = 0.05
        difx=center.x-point.x
        dify=center.y-point.y
        if difx > umbralChico and difx<umbralGrande:
            if dify> umbralGrande:
                # print("Norte Izquierda")
                self.tokensNorth[0]=token
            elif dify<-umbralGrande:
                # print("Sur Izquierda")
                self.tokensSouth[0]=token
            else:
                # print("Interna izquierda")
                self.tokensHorizontalInternalWall[0]=token

        elif difx>umbralGrande:
            if dify>umbralChico:
                # print("Oeste Arriba")
                self.tokensWest[0]=token
            elif dify<-umbralChico:
                # print("Oeste Abajo")
                self.tokensWest[2]=token
            else:
                # print("Oeste Medio")
                # ACAACA SI NO LO ARREGLAN, LO VAMOS A PONER ARRIBA
                # self.tokensWest[1]=token # ACAACA ESTO ES LO CORRECTO!!!!!!!!
                self.tokensWest[0]=token
        elif difx<=umbralChico and difx>=-umbralChico:
            if dify>umbralGrande:
                # print("Norte Medio")
                # ACAACA SI NO LO ARREGLAN, LO VAMOS A PONER A LA IZQUIERDA
                # self.tokensNorth[1]=token # ACAACA ESTO ES LO CORRECTO!!!!!!!!
                self.tokensNorth[0]=token
            elif dify>=umbralChico and dify<=umbralGrande:
                # print("Interna Arriba")
                self.tokensVerticalInternalWall[0]=token
            elif dify<=-umbralChico and dify>=-umbralGrande:
                # print("Interna Abajo")
                self.tokensVerticalInternalWall[1]=token
            else:
                # print("Sur Medio")
                # ACAACA SI NO LO ARREGLAN, LO VAMOS A PONER A LA IZQUIERDA
                # self.tokensSouth[1]=token # ACAACA ESTO ES LO CORRECTO!!!!!!!!
                self.tokensSouth[0]=token
        elif difx<-umbralChico and difx>-umbralGrande:
            if dify>umbralGrande:
                # print("Norte derecha")
                self.tokensNorth[2]=token
            elif dify<-umbralGrande:
                # print("Sur Derecha")
                self.tokensSouth[2]=token
            else:
                # print("Interna derecha")
                self.tokensHorizontalInternalWall[1]=token
        else:
            if dify>umbralChico:
                # print("Este Arriba")
                self.tokensEast[0]=token
            elif dify<-umbralChico:
                # print("Este Abajo")
                self.tokensEast[2]=token
            else:
                # print("Este Medio")
                # ACAACA SI NO LO ARREGLAN, LO VAMOS A PONER ARRIBA
                #self.tokensEast[1]=token # ACAACA ESTO ES LO CORRECTO!!!!!!!!
                self.tokensEast[0]=token
            