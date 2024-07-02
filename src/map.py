from point import Point

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
            tile = Tile(col, row)
            self.tiles[(col, row)] = tile
        return tile
            
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
                    elif t.isBlackHole:
                        chars[1][1] = "B"
                    elif t.isBlue:
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
    
class Tile:
    WIDTH = 0.12  
    HEIGHT = 0.12

    def __init__(self, col, row) -> None:
        self.col = col
        self.row = row
        self.visits = 0
        
        self.north = [-1, -1, -1]
        self.west = [-1, -1, -1]
        self.east = [-1, -1, -1]
        self.south = [-1, -1, -1]

        self.isBlackHole = False
        self.isSwamp = False
        self.isCheckpoint = False
        self.isPurple = False
        self.isRed = False
        self.isGreen = False
        self.isBlue = False

        self.hasObstacle = False

        self.isOrange = False
        self.isYellow = False
        self.area = None
    
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
    
    def get_color(self):
        if self.isBlue:
            return 'Blue'
        elif self.isPurple:
            return 'Purple'
        elif self.isYellow:
            return 'Yellow'
        elif self.isGreen:
            return 'Green'
        elif self.isRed:
            return 'Red'
        elif self.isOrange:
            return 'Orange'
        else:
            return None