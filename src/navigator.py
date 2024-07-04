
class Navigator:

    def whereToGo(self, robot):
        return robot.position
    
class Navigator1(Navigator):

    def whereToGo(self, robot):
        tiles = robot.checkNeighbours()
        tiles.sort(key=lambda t: t.visits)
        tile = tiles[0]
        return robot.map.gridToPosition(tile.col, tile.row)
    
class Navigator2(Navigator):
    pass