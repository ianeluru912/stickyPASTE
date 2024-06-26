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

    def addTile(self, col, row):
        tile = self.getTileAt(col, row)
        if tile == None:
            tile = Tile(col, row)
            self.tiles[(col, row)] = tile
        return tile

    def getTileAt(self, col, row):
        return self.tiles.get((col, row))
            
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
        self.north = None
        self.west = None
        self.east = None
        self.south = None
        self.isBlackHole = False
        self.isSwamp = False
        self.isCheckpoint = False
        self.isPurple = False
        self.isRed = False
        self.isGreen = False
        self.isBlue = False
    def isConnectedTo(self, tile):
        if self.north == tile: 
            return True
        elif self.west == tile:
            return True
        elif self.east == tile:
            return True
        elif self.south == tile:
            return True
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
