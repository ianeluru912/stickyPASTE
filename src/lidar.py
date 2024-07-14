import math

class Lidar:
    shift ={'N': 0, 'E': 384, 'S': 256, 'W':128}

    rays_1 ={0:[210, 0.095, 0.12],1:[210, 0.06, 0.08], 2:[256, 0.05, 0.075], 3:[302, 0.06, 0.08],\
            4: [302, 0.095, 0.12], 5: [338, 0.095, 0.12], 6:[338,0.06, 0.085], 7:[384,0.045, 0.075], \
            8:[430,0.06, 0.085],9:[430,0.095, 0.12],10:[466,0.095, 0.12],11:[466,0.06, 0.085], 12:[0,0.05, 0.075], \
            13:[46,0.06, 0.085], 14:[46,0.095, 0.12], 15:[82,0.095, 0.12], 16:[82,0.06, 0.085], 17:[128,0.045, 0.075], \
            18:[174,0.06, 0.085], 19:[174,0.095, 0.12]}
    """rays_1 ={0:[210, 0.095, 0.12],1:[210, 0.06, 0.08], 2:[256, 0.05, 0.075], 3:[299, 0.06, 0.08],\
            4: [302, 0.095, 0.12], 5: [338, 0.095, 0.12], 6:[338, 0.06,], 7:[0, 0,0], \
            8:[43, 0.06, 0.08],9:[86,0.057, 0.08],10:[128,0,0],11:[171,0.057, 0.08]}"""
    
    rays_2 ={0: [230,0.112, 0.1354], 1: [256, 0.05, 0.075], 2: [282,0.112, 0.1354], 3:[302,0.095, 0.12], 4:[302,0.06, 0.085], 5:[340,0.057, 0.077], 6:[428,0.057, 0.077],\
            7:[466,0.06, 0.085], 8:[466,0.095, 0.12], 9:[486,0.112, 0.1354], 10:[0,0.05, 0.075], 11:[26,0.112, 0.1354], 12:[46,0.095, 0.12], 13:[46,0.06, 0.085],\
            14:[84,0.057, 0.077], 15:[172,0.057, 0.077], 16:[210,0.06, 0.085], 17:[210,0.095, 0.12]}
    
    rays_3 ={0: [174,0.095, 0.12], 1: [174,0.06, 0.085], 2: [212,0.057, 0.077], 3:[300,0.057, 0.077], 4:[338,0.06, 0.085], 5:[338,0.095, 0.12], 6:[358,0.112, 0.135],\
            7:[384,0.045, 0.075], 8:[410,0.112, 0.135], 9:[430,0.095, 0.12], 10:[430,0.06, 0.085], 11:[468,0.057, 0.077], 12:[44,0.057, 0.077], 13:[82,0.06, 0.085],\
            14:[82,0.095, 0.12], 15:[102,0.112, 0.1354], 16:[128,0.045, 0.075], 17:[154,0.112, 0.135]}
    
    rays_4 = {0: [205, 0.13, 0.16], 1: [205, 0.087, 0.12], 2: [235, 0.112, 0.1354], 3: [256, 0.045, 0.075], 4: [277, 0.112, 0.1354], \
              5: [307, 0.087, 0.12], 6: [307, 0.13, 0.16], 7: [333, 0.13, 0.16], 8: [333, 0.085, 0.12], 9: [363, 0.112, 0.135], \
              10: [384, 0.045, 0.075], 11: [405, 0.112, 0.135], 12: [435, 0.085, 0.12], 13: [435, 0.13, 0.16], 14: [461, 0.13, 0.16],\
              15: [461, 0.087, 0.12], 16: [491, 0.112, 0.1354], 17: [0, 0.045, 0.075], 18: [21, 0.112, 0.1354], 19: [51, 0.087, 0.12], \
              20: [51, 0.13, 0.16], 21: [77, 0.13, 0.16], 22: [77, 0.085, 0.12], 23: [107, 0.112, 0.135], 24: [128, 0.045, 0.075], \
              25: [149, 0.112, 0.135], 26: [179, 0.085, 0.12], 27: [179, 0.13, 0.16], 28: [235, 0.05, 0.077], 29: [277, 0.05, 0.077],\
              30: [363, 0.05, 0.075], 31: [405, 0.05, 0.075], 32: [491, 0.05, 0.077], 33: [21, 0.05, 0.077], 34: [107,0.05, 0.075],\
              35: [149,0.05, 0.075]}
    
    def __init__(self, lidar, time_step):
        self.lidar = lidar
        self.lidar.enable(time_step)
        self.rangeImage = None
    
    def update(self):
        self.rangeImage = self.lidar.getRangeImage()[1024:1536]

    def isOpenNorth(self, orient):
        lidar_idx = {'N': 256,
                     'W': 384,
                     'S': 0,
                     'E': 128}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
    
    def isOpenSouth(self, orient):
        lidar_idx = {'S': 256,
                     'E': 384,
                     'N': 0,
                     'W': 128}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08

    def isOpenWest(self, orient):
        lidar_idx = {'S': 384,
                     'E': 0,
                     'N': 128,
                     'W': 256}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
    
    def isOpenEast(self, orient):
        lidar_idx = {'S': 128,
                     'E': 256,
                     'N': 384,
                     'W': 0}
        
        dist = self.rangeImage[lidar_idx[orient]]
        return dist >= 0.08
    
    def ver_walls(self, rotation): # caso 4
        shift = self.rotToLidar(rotation)
        # gira los rayos para que estén en orientación Norte
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        walls = {}
        for i in self.rays_4.keys():
            walls[i]=(rangeLocal[self.rays_4[i][0]])
        return walls

    def get_walls_1(self, rotation):
        shift = self.rotToLidar(rotation)
        # gira los rayos para que estén en orientación Norte
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        # create a dictionary with the walls
        walls = {}
        for i in self.rays_1.keys():
            ray=self.rays_1[i][0]
            rayDistance=rangeLocal[ray]
            lowerLimit=self.rays_1[i][1]
            upperLimit=self.rays_1[i][2]

            if rayDistance>=lowerLimit and rayDistance<=upperLimit:
                walls[i]=1
                # walls[i]=(1, lowerLimit,upperLimit,rayDistance)
            elif rayDistance<lowerLimit:
                walls[i]=-1
                # walls[i]=(-1,lowerLimit,upperLimit,rayDistance)
            else:
                walls[i]=0
                # walls[i]=(0,lowerLimit,upperLimit,rayDistance)
        if walls[3] == 1:
            walls[2] = -1
            walls[4] = -1
        if walls[1] == 1:
            walls[0] = -1
            walls[2] = -1
        if walls[6] == 1:
            walls[5] = -1
            walls[7] = -1
        if walls[8] == 1:
            walls[7] = -1
            walls[9] = -1
        if walls[11] == 1:
            walls[10] = -1
            walls[12] = -1
        if walls[13] == 1:
            walls[12] = -1
            walls[14] = -1
        if walls[16] == 1:
            walls[15] = -1
            walls[17] = -1
        if walls[18] == 1:
            walls[17] = -1
            walls[19] = -1
            #walls.append(rangeLocal[self.rays_1[i][0]])
        return walls
    
    def updateWalls1(self, rotation, map, tile):
        walls = self.get_walls_1(rotation)

        self.setWall(tile.north, 0, walls[1])
        self.setWall(tile.north, 1, 0)
        self.setWall(tile.north, 2, walls[3])

        self.setWall(tile.east, 0, walls[6])
        self.setWall(tile.east, 1, 0)
        self.setWall(tile.east, 2, walls[8])

        self.setWall(tile.south, 0, walls[11])
        self.setWall(tile.south, 1, 0)
        self.setWall(tile.south, 2, walls[13])

        self.setWall(tile.west, 0, walls[16])
        self.setWall(tile.west, 1, 0)
        self.setWall(tile.west, 2, walls[18])

        north_tile = tile.getNorthTile()
        east_tile = tile.getEastTile()
        west_tile = tile.getWestTile()
        south_tile = tile.getSouthTile()

        self.setWall(north_tile.west, 0, walls[0])

        self.setWall(north_tile.south, 0, walls[3])
        self.setWall(north_tile.south, 1, walls[2])
        self.setWall(north_tile.south, 2, walls[1])

        self.setWall(north_tile.east, 2, walls[4])

        self.setWall(east_tile.north, 0, walls[5])

        self.setWall(east_tile.west, 0, walls[8])
        self.setWall(east_tile.west, 1, walls[7])
        self.setWall(east_tile.west, 2, walls[6])

        self.setWall(east_tile.south, 2, walls[9])

        self.setWall(south_tile.east, 0, walls[10])

        self.setWall(south_tile.north, 0, walls[13])
        self.setWall(south_tile.north, 1, walls[12])
        self.setWall(south_tile.north, 2, walls[11])

        self.setWall(south_tile.west, 2, walls[14])
        
        self.setWall(west_tile.south, 0, walls[15])

        self.setWall(west_tile.east, 0, walls[18])
        self.setWall(west_tile.east, 1, walls[17])
        self.setWall(west_tile.east, 2, walls[16])

        self.setWall(west_tile.north, 2, walls[19])

        self.fixNeighbours(tile)
        self.fixNeighbours(north_tile)
        self.fixNeighbours(south_tile)
        self.fixNeighbours(east_tile)
        self.fixNeighbours(west_tile)
    
    def get_walls_2(self, rotation):
        shift = self.rotToLidar(rotation)
        # gira los rayos para que estén en orientación Norte
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        walls = {}
        for i in self.rays_2.keys():
            ray=self.rays_2[i][0]
            rayDistance=rangeLocal[ray]
            lowerLimit=self.rays_2[i][1]
            upperLimit=self.rays_2[i][2]
            if rayDistance>=lowerLimit and rayDistance<=upperLimit:
                walls[i]=1
                # walls[i]=(1, lowerLimit,upperLimit,rayDistance)
            elif rayDistance<lowerLimit:
                walls[i]=-1
                # walls[i]=(-1,lowerLimit,upperLimit,rayDistance)
            else:
                walls[i]=0
                # walls[i]=(0,lowerLimit,upperLimit,rayDistance)
        if walls[4]==1:
            walls[1]=-1
            walls[2]=-1
            walls[3]=-1
        if walls[7]==1:
            walls[8]=-1
            walls[9]=-1
            walls[10]=-1
        if walls[13]==1:
            walls[10]=-1
            walls[11]=-1
            walls[12]=-1
        if walls[16]==1:
            walls[17]=-1
            walls[0]=-1
            walls[1]=-1    
        return walls

    def updateWalls2(self, rotation, map, tiles):
        walls = self.get_walls_2(rotation)
        north_tile = None
        south_tile = None
        if tiles[0].row < tiles[1].row:
            north_tile = tiles[0]
            south_tile = tiles[1]
        else:
            north_tile = tiles[1]
            south_tile = tiles[0]

        self.setWall(north_tile.north, 0, walls[0])
        self.setWall(north_tile.north, 1, walls[1])
        self.setWall(north_tile.north, 2, walls[2])

        self.setWall(north_tile.east, 0, walls[3])
        self.setWall(north_tile.east, 1, walls[4])
        self.setWall(north_tile.east, 2, walls[5])

        north_tile.south = [0, 0, 0]

        self.setWall(north_tile.west, 0, walls[15])
        self.setWall(north_tile.west, 1, walls[16])
        self.setWall(north_tile.west, 2, walls[17])

        south_tile.north = [0, 0, 0]

        self.setWall(south_tile.east, 0, walls[6])
        self.setWall(south_tile.east, 1, walls[7])
        self.setWall(south_tile.east, 2, walls[8])

        self.setWall(south_tile.south, 0, walls[9])
        self.setWall(south_tile.south, 1, walls[10])
        self.setWall(south_tile.south, 2, walls[11])

        self.setWall(south_tile.west, 0, walls[12])
        self.setWall(south_tile.west, 1, walls[13])
        self.setWall(south_tile.west, 2, walls[14])

        self.fixNeighbours(north_tile)
        self.fixNeighbours(south_tile)
        
    def get_walls_3(self, rotation):
        shift = self.rotToLidar(rotation)
        # gira los rayos para que estén en orientación Norte
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        walls = {}
        for i in self.rays_3.keys():
            ray=self.rays_3[i][0]
            rayDistance=rangeLocal[ray]
            lowerLimit=self.rays_3[i][1]
            upperLimit=self.rays_3[i][2]
            if rayDistance>=lowerLimit and rayDistance<=upperLimit:
                walls[i]= 1
                # walls[i]=(1, lowerLimit,upperLimit,rayDistance)
            elif rayDistance<lowerLimit:
                walls[i]= -1
                # walls[i]=(-1,lowerLimit,upperLimit,rayDistance)
            else:
                walls[i]= 0
                # walls[i]=(0,lowerLimit,upperLimit,rayDistance)
        if walls[1]==1:
            walls[0]=-1
            walls[16]=-1
            walls[17]=-1
        if walls[4]==1:
            walls[5]=-1
            walls[6]=-1
            walls[7]=-1
        if walls[10]==1:
            walls[7]=-1
            walls[8]=-1
            walls[9]=-1
        if walls[13]==1:
            walls[14]=-1
            walls[15]=-1
            walls[16]=-1
        return walls
    
    def updateWalls3(self, rotation, map, tiles):        
        walls = self.get_walls_3(rotation)
        
        west_tile = None
        east_tile = None
        if tiles[0].col < tiles[1].col:
            west_tile = tiles[0]
            east_tile = tiles[1]
        else:
            west_tile = tiles[1]
            east_tile = tiles[0]

        self.setWall(west_tile.north, 0, walls[0])
        self.setWall(west_tile.north, 1, walls[1])
        self.setWall(west_tile.north, 2, walls[2])

        west_tile.east = [0, 0, 0]

        self.setWall(west_tile.south, 0, walls[12])
        self.setWall(west_tile.south, 1, walls[13])
        self.setWall(west_tile.south, 2, walls[14])

        self.setWall(west_tile.west, 0, walls[15])
        self.setWall(west_tile.west, 1, walls[16])
        self.setWall(west_tile.west, 2, walls[17])

        self.setWall(east_tile.north, 0, walls[3])
        self.setWall(east_tile.north, 1, walls[4])
        self.setWall(east_tile.north, 2, walls[5])
        
        self.setWall(east_tile.east, 0, walls[6])
        self.setWall(east_tile.east, 1, walls[7])
        self.setWall(east_tile.east, 2, walls[8])

        self.setWall(east_tile.south, 0, walls[9])
        self.setWall(east_tile.south, 1, walls[10])
        self.setWall(east_tile.south, 2, walls[11])

        east_tile.west = [0, 0, 0]

        self.fixNeighbours(west_tile)
        self.fixNeighbours(east_tile)
    
    def get_walls_4(self, rotation):
        # print(self.ver_walls(rotation))
        shift = self.rotToLidar(rotation)
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        walls = {}
        for i in self.rays_4.keys():
            ray=self.rays_4[i][0]
            rayDistance=rangeLocal[ray]
            lowerLimit=self.rays_4[i][1]
            upperLimit=self.rays_4[i][2]
            if rayDistance>=lowerLimit and rayDistance<=upperLimit:
                walls[i]= 1
            elif rayDistance<lowerLimit:
                walls[i]= -1
            else:
                walls[i]= 0
        if walls[1] == 1:
            walls[0] = -1
        if walls[28] == 1:
            walls[0] = -1
            walls[1] = -1
            walls[2] = -1
            walls[3] = -1
        if walls[29] == 1:
            walls[3] = -1
            walls[4] = -1
            walls[5] = -1
            walls[6] = -1
        if walls[30] == 1:
            walls[10] = -1
            walls[7] = -1
            walls[8] = -1
            walls[9] = -1
        if walls[31] == 1:
            walls[10] = -1
            walls[11] = -1
            walls[12] = -1
            walls[13] = -1
        if walls[32] == 1:
            walls[16] = -1
            walls[17] = -1
            walls[14] = -1
            walls[15] = -1
        if walls[33] == 1:
            walls[17] = -1
            walls[18] = -1
            walls[19] = -1
            walls[20] = -1
        if walls[34] == 1:
            walls[21] = -1
            walls[22] = -1
            walls[23] = -1
            walls[24] = -1
        if walls[35] == 1:
            walls[24] = -1
            walls[25] = -1
            walls[26] = -1
            walls[27] = -1
        if walls[8] == 1:
            walls[7] = -1
        if walls[12] == 1:
            walls[13] = -1
        if walls[15] == 1:
            walls[14] = -1
        if walls[19] == 1:
            walls[20] = -1
        if walls[22] == 1:
            walls[21] = -1
        if walls[26] == 1:
            walls[27] = -1
        return walls
        
    def updateWalls4(self, rotation, map, tiles):
        walls = self.get_walls_4(rotation)

        min_col = math.inf
        min_row = math.inf
        for t in tiles:
            if t.col < min_col: min_col = t.col
            if t.row < min_row: min_row = t.row

        nw_tile = map.getTileAt(min_col, min_row)
        ne_tile = map.getTileAt(min_col + 1, min_row)
        sw_tile = map.getTileAt(min_col, min_row + 1)
        se_tile = map.getTileAt(min_col + 1, min_row + 1)

        self.setWall(nw_tile.north, 0, walls[0])        
        self.setWall(nw_tile.north, 1, walls[1])
        self.setWall(nw_tile.north, 2, walls[2])

        self.setWall(nw_tile.east, 0, walls[3])
        self.setWall(nw_tile.east, 1, walls[28])
        self.setWall(nw_tile.east, 2, 0)

        self.setWall(nw_tile.south, 0, 0)
        self.setWall(nw_tile.south, 1, walls[35])
        self.setWall(nw_tile.south, 2, walls[24])

        self.setWall(nw_tile.west, 0, walls[25])
        self.setWall(nw_tile.west, 1, walls[26])
        self.setWall(nw_tile.west, 2, walls[27])

        self.setWall(ne_tile.north, 0, walls[4])
        self.setWall(ne_tile.north, 1, walls[5])
        self.setWall(ne_tile.north, 2, walls[6])

        self.setWall(ne_tile.east, 0, walls[7])
        self.setWall(ne_tile.east, 1, walls[8])
        self.setWall(ne_tile.east, 2, walls[9])

        self.setWall(ne_tile.south, 0, walls[10]) #repite
        self.setWall(ne_tile.south, 1, walls[30])
        self.setWall(ne_tile.south, 2, 0)

        self.setWall(ne_tile.west, 0, 0) 
        self.setWall(ne_tile.west, 1, walls[29])
        self.setWall(ne_tile.west, 2, walls[3]) #repite

        self.setWall(sw_tile.north, 0, walls[24])
        self.setWall(sw_tile.north, 1, walls[34])
        self.setWall(sw_tile.north, 2, 0)

        self.setWall(sw_tile.east, 0, 0)
        self.setWall(sw_tile.east, 1, walls[33])
        self.setWall(sw_tile.east, 2, walls[17])

        self.setWall(sw_tile.south, 0, walls[18])
        self.setWall(sw_tile.south, 1, walls[19])
        self.setWall(sw_tile.south, 2, walls[20])

        self.setWall(sw_tile.west, 0, walls[21])
        self.setWall(sw_tile.west, 1, walls[22])
        self.setWall(sw_tile.west, 2, walls[23])

        self.setWall(se_tile.north, 0, 0)
        self.setWall(se_tile.north, 1, walls[31])
        self.setWall(se_tile.north, 2, walls[10])

        self.setWall(se_tile.east, 0, walls[11])
        self.setWall(se_tile.east, 1, walls[12])
        self.setWall(se_tile.east, 2, walls[13])

        self.setWall(se_tile.south, 0, walls[14])
        self.setWall(se_tile.south, 1, walls[15])
        self.setWall(se_tile.south, 2, walls[16])

        self.setWall(se_tile.west, 0, walls[17]) #repite
        self.setWall(se_tile.west, 1, walls[32])
        self.setWall(se_tile.west, 2, 0)

        self.fixNeighbours(nw_tile)
        self.fixNeighbours(ne_tile)
        self.fixNeighbours(sw_tile)
        self.fixNeighbours(se_tile)

    def setWall(self, tile_wall, idx, value):
        if value < 0: return 
        
        # si ya hay un 1 o un 0, en una pared, confiamos en el valor que ya tiene (Gonzalo)
        # if tile_wall[idx] == 0 and value != 0:
        #     print("VEO PARED DONDE ANTES NO VEIA")
        # if tile_wall[idx] == 1 and value != 1:
        #     print("NO VEO PARED DONDE ANTES SI VEIA")
            
        tile_wall[idx] = value 

    def fixNeighbours(self, tile):
        if tile.north[0] != -1:
            tile.getNorthTile().south[2] = tile.north[0]
        if tile.north[2] != -1:
            tile.getNorthTile().south[0] = tile.north[2]
        if tile.east[0] != -1:
            tile.getEastTile().west[2] = tile.east[0]
        if tile.east[2] != -1:
            tile.getEastTile().west[0] = tile.east[2]
        if tile.west[0] != -1:
            tile.getWestTile().east[2] = tile.west[0]
        if tile.west[2] != -1:
            tile.getWestTile().east[0] = tile.west[2]
        if tile.south[0] != -1:
            tile.getSouthTile().north[2] = tile.south[0]
        if tile.south[2] != -1:
            tile.getSouthTile().north[0] = tile.south[2]

    def rotToLidar(self, rot):
        #Cuánto girar los rayos para tomar referencia norte
        return int((256/math.pi)*rot)%512
    
    def hayAlgoIzquierda(self):
        leftDist = self.rangeImage[128]
        return leftDist < 0.08

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128*3]
        return rightDist < 0.08

    def getNearestObstacle(self):
        threshold = 0.05
        min_dist = math.inf
        min_ray = None
        # Si achicamos el offset corremos el riesgo de quedarnos trabados por un 
        # obstáculo, mejor ser conservador...
        offset = 60
        for ray_idx in range(256-offset, 257+offset):
            dist = self.rangeImage[ray_idx]
            if dist < threshold and dist < min_dist:
                min_dist = dist
                min_ray = ray_idx
        return (min_ray, min_dist)
            