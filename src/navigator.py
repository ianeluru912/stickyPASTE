from map import TileType

class Navigator:

    def whereToGo(self, robot):
        return robot.position
    
class Navigator1(Navigator):

    def whereToGo(self, robot):
        tiles = self.checkNeighbours(robot)
        tiles.sort(key=lambda t: t.visits)
        tile = tiles[0]
        return robot.map.gridToPosition(tile.col, tile.row)
    
    def checkNeighbours(self, robot):
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
            if current_tile.isConnectedTo(tile) and not tile.type== TileType.BLACK_HOLE and not tile.hasObstacle:
                tiles.append(tile)
        return tiles
    
class Navigator2(Navigator):
    pass