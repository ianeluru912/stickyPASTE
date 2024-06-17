class Map:
    def __init__(self, origin) -> None:
        self.origin = origin
        self.tiles = {}

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