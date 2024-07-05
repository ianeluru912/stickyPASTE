from math import pi as PI
class Lidar:
    shift ={'N': 0, 'E': 384, 'S': 256, 'W':128}
    rays_1 ={0:(214,0.053, 0.073),1:(256,0,0), 2:(299, 0.053, 0.073), 3:(342,0.053, 0.073),\
             4: (384,0, 0), 5: (427, 0.053, 0.073), 6:(470, 0.053, 0.073), 7:(0, 0,0), \
                8:(43, 0.053, 0.073),9:(86,0.053, 0.073),10:(128,0,0),11:(171,0.053, 0.073)}
    
    rays_2 ={0: (230,0.11,0.13), 1: (256,0.04,0.072), 2: (282,0.11,0.13), 3:(302,0.093,0.113), 4:(302,0.055,0.075), 5:(340,0.054,0.074), 6:(428,0.054,0.074),\
            7:(466,0.055,0.075), 8:(466,0.093,0.113), 9:(486,0.11,0.13), 10:(0,0.052,0.072), 11:(26,0.11,0.13), 12:(46,0.093,0.113), 13:(46,0.055,0.075),\
            14:(84,0.054,0.074), 15:(172,0.054,0.074), 16:(210,0.055,0.075), 17:(210,0.093,0.113)}
    
    rays_3 ={0: (174,0.09, 0.113), 1: (174,0.055, 0.075), 2: (212,0.055, 0.075), 3:(300,0.055, 0.075), 4:(338,0.055, 0.075), 5:(338,0.09, 0.113), 6:(358,0.012, 0.13),\
            7:(384,0.05, 0.07), 8:(410,0.012, 0.13), 9:(430,0.09, 0.113), 10:(430,0.055, 0.075), 11:(468,0.055, 0.075), 12:(44,0.055, 0.075), 13:(82,0.055, 0.075),\
            14:(82,0.09, 0.113), 15:(102,0.012, 0.13), 16:(128,0.05, 0.07), 17:(154,0.012, 0.13)}
    
    rays_4 = {0: (205, 0.04, 0.24), 1: (205, 0.09, 0.11), 2: (235, 0.01, 0.21), 3: (256, 0.049, 0.069), 4: (277, 0.01, 0.21), \
              5: (307, 0.09, 0.11), 6: (307, 0.04, 0.24), 7: (333, 0.133, 0.1533), 8: (333, 0.086, 0.106), 9: (363, 0.01, 0.21), \
              10: (384, 0.049, 0.069), 11: (405, 0.01, 0.21), 12: (435, 0.086, 0.106), 13: (435, 0.133, 0.1533), 14: (461, 0.04, 0.24),\
              15: (461, 0.09, 0.11), 16: (491, 0.01, 0.21), 17: (0, 0.049, 0.06), 18: (21, 0.01, 0.21), 19: (51, 0.09, 0.11), \
              20: (51, 0.04, 0.24), 21: (77, 0.133, 0.1533), 22: (77, 0.086, 0.106), 23: (107, 0.01, 0.21), 24: (128, 0, 0), \
              25: (149, 0.01, 0.21), 26: (179, 0.086, 0.106), 27: (179, 0.133, 0.1533), 28: (235, 0.046, 0.066), 29: (277, 0.046, 0.066),\
              30: (363, 0.043, 0.063), 31: (405, 0.043, 0.063), 32: (491, 0.046, 0.066), 33: (21, 0.046, 0.066), 34: (107, 0.043, 0.063),\
              35: (149, 0.043, 0.063)}
    
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
    
    def ver_walls(self, rotation): # caso 3
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
                # add wall witk key i and value 1
                walls[i]=1
            else:
                walls[i]=0
            # walls.append(rangeLocal[self.rays_1[i][0]])
        return walls
    
    def updateWalls1(self, rotation, map, tile):
        walls = self.get_walls_1(rotation)
        
        # Current tile!
        tile.north[0] = walls[0]
        tile.north[1] = walls[1]
        tile.north[2] = walls[2]
        
        tile.east[0] = walls[3]
        tile.east[1] = walls[4]
        tile.east[2] = walls[5]
        
        tile.south[0] = walls[6]
        tile.south[1] = walls[7]
        tile.south[2] = walls[8]
        
        tile.west[0] = walls[9]
        tile.west[1] = walls[10]
        tile.west[2] = walls[11]
        
        self.fixNeighbours(map, tile)
    
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

        north_tile.north[0] = walls[0]
        north_tile.north[1] = walls[1]
        north_tile.north[2] = walls[2]

        north_tile.east[0] = walls[3]
        north_tile.east[1] = walls[4]
        north_tile.east[2] = walls[5]

        north_tile.south = [0, 0, 0]

        north_tile.west[0] = walls[15]
        north_tile.west[1] = walls[16]
        north_tile.west[2] = walls[17]

        south_tile.north = [0, 0, 0]

        south_tile.east[0] = walls[6]
        south_tile.east[1] = walls[7]
        south_tile.east[2] = walls[8]

        south_tile.south[0] = walls[9]
        south_tile.south[1] = walls[10]
        south_tile.south[2] = walls[11]

        south_tile.west[0] = walls[12]
        south_tile.west[1] = walls[13]
        south_tile.west[2] = walls[14]

        self.fixNeighbours(map, north_tile)
        self.fixNeighbours(map, south_tile)
        
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

        west_tile.north[0] = walls[0]
        west_tile.north[1] = walls[1]
        west_tile.north[2] = walls[2]

        west_tile.east = [0, 0, 0]

        west_tile.south[0] = walls[12]
        west_tile.south[1] = walls[13]
        west_tile.south[2] = walls[14]

        west_tile.west[0] = walls[15]
        west_tile.west[1] = walls[16]
        west_tile.west[2] = walls[17]

        east_tile.north[0] = walls[3]
        east_tile.north[1] = walls[4]
        east_tile.north[2] = walls[5]
        
        east_tile.east[0] = walls[6]
        east_tile.east[1] = walls[7]
        east_tile.east[2] = walls[8]

        east_tile.south[0] = walls[9]
        east_tile.south[1] = walls[10]
        east_tile.south[2] = walls[11]

        east_tile.west = [0, 0, 0]

        self.fixNeighbours(map, west_tile)
        self.fixNeighbours(map, east_tile)
    
    def get_walls_4(self, rotation):
        print(dict(sorted(self.rays_4.items())))
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
        if walls[1]==1:
            walls[0]=-1
        if walls[28]==1:
            walls[0]=-1
            walls[1]=-1
            walls[2]=-1
            walls[3]=-1
        if walls[29]==1:
            walls[3]=-1
            walls[4]=-1
            walls[5]=-1
            walls[6]=-1
        if walls[30]==1:
            walls[10]=-1
            walls[7]=-1
            walls[8]=-1
            walls[9]=-1
        if walls[31]==1:
            walls[10]=-1
            walls[11]=-1
            walls[12]=-1
            walls[13]=-1
        if walls[32]==1:
            walls[16]=-1
            walls[17]=-1
            walls[14]=-1
            walls[15]=-1
        if walls[33]==1:
            walls[17]=-1
            walls[18]=-1
            walls[19]=-1
            walls[20]=-1
        if walls[34]==1:
            walls[21]=-1
            walls[22]=-1
            walls[23]=-1
            walls[24]=-1
        if walls[35]==1:
            walls[24]=-1
            walls[25]=-1
            walls[26]=-1
            walls[27]=-1
        if walls[8]==1:
            walls[7]=-1
        if walls[12]==1:
            walls[13]=-1
        if walls[15]==1:
            walls[14]=-1
        if walls[19]==1:
            walls[20]=-1
        if walls[22]==1:
            walls[21]=-1
        if walls[26]==1:
            walls[27]=-1
        return walls
    
    def updateWalls4(self, rotation, map, tiles):
        walls = self.get_walls_4(rotation)
        tiles.sort(key=lambda t: t.col + t.row)
        nw_tile = tiles[0]
        ne_tile = tiles[1]
        sw_tile = tiles[2]
        se_tile = tiles[3]

        nw_tile.north[0] = walls[0]
        nw_tile.north[1] = walls[1]
        nw_tile.north[2] = walls[2]

        nw_tile.east[0] = walls[3]
        nw_tile.east[1] = walls[28]
        nw_tile.east[2] = [0]

        nw_tile.south[0] = walls[24]
        nw_tile.south[1] = walls[35]
        nw_tile.south[2] = [0]

        nw_tile.west[0] = walls[25]
        nw_tile.west[1] = walls[26]
        nw_tile.west[2] = walls[27]

        ne_tile.north[0] = walls[4]
        ne_tile.north[1] = walls[5]
        ne_tile.north[2] = walls[6]

        ne_tile.east[0] = walls[7]
        ne_tile.east[1] = walls[8]
        ne_tile.east[2] = walls[9]

        ne_tile.south[0] = walls[10]
        ne_tile.south[1] = walls[30]
        ne_tile.south[2] = [0]

        ne_tile.west[0] = walls[3]
        ne_tile.west[1] = walls[29]
        ne_tile.west[2] = [0]

        sw_tile.north[0] = walls[24]
        sw_tile.north[1] = walls[34]
        sw_tile.north[2] = [0]

        sw_tile.east[0] = walls[17]
        sw_tile.east[1] = walls[33]
        sw_tile.east[2] = [0]

        sw_tile.south[0] = walls[18]
        sw_tile.south[1] = walls[19]
        sw_tile.south[2] = walls[20]

        sw_tile.west[0] = walls[21]
        sw_tile.west[1] = walls[22]
        sw_tile.west[2] = walls[23]

        se_tile.north[0] = walls[10]
        se_tile.north[1] = walls[31]
        se_tile.north[2] = [0]

        se_tile.east[0] = walls[11]
        se_tile.east[1] = walls[12]
        se_tile.east[2] = walls[13]

        se_tile.south[0] = walls[14]
        se_tile.south[1] = walls[15]
        se_tile.south[2] = walls[16]

        se_tile.west[0] = walls[17]
        se_tile.west[1] = walls[32]
        se_tile.west[2] = [0]

        self.fixNeighbours(map, nw_tile)
        self.fixNeighbours(map, ne_tile)
        self.fixNeighbours(map, sw_tile)
        self.fixNeighbours(map, se_tile)
        # TODO(Martu): Verificar que los tiles sean los correctos y terminar de 
        # marcar las paredes

    def fixNeighbours(self, map, tile):
        north_tile = map.getTileAt(tile.col, tile.row - 1)
        east_tile = map.getTileAt(tile.col + 1, tile.row)
        west_tile = map.getTileAt(tile.col - 1, tile.row)
        south_tile = map.getTileAt(tile.col, tile.row + 1)

        if tile.north[0] != -1:
            north_tile.south[2] = tile.north[0]
        if tile.north[2] != -1:
            north_tile.south[0] = tile.north[2]
        if tile.east[0] != -1:
            east_tile.west[2] = tile.east[0]
        if tile.east[2] != -1:
            east_tile.west[0] = tile.east[2]
        if tile.west[0] != -1:
            west_tile.east[2] = tile.west[0]
        if tile.west[2] != -1:
            west_tile.east[0] = tile.west[2]
        if tile.south[0] != -1:
            south_tile.north[2] = tile.south[0]
        if tile.south[2] != -1:
            south_tile.north[0] = tile.south[2]

    def rotToLidar(self, rot):
        #Cuánto girar los rayos para tomar referencia norte
        return int((256/PI)*rot)%512
    
    def hayAlgoIzquierda(self):
        leftDist = self.rangeImage[128]
        return leftDist < 0.08

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128*3]
        return rightDist < 0.08
    
    def is_obstacle_preventing_passage(self):
        rays = self.rangeImage[224:288]
        not_available_rays = filter(lambda x: x < 0.05, rays)
        if len(list(not_available_rays)) >= 1:
            print('obstacle is not allowing passage')
            return True
        return False
