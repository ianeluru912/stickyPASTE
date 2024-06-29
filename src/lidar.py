from math import pi as PI
class Lidar:
    shift ={'N': 0, 'E': 384, 'S': 256, 'W':128}
    rays_1 ={0:(214,0.053, 0.073),1:(256,0,0), 2:(299, 0.053, 0.073), 3:(342,0.053, 0.073),\
             4: (384,0, 0), 5: (427, 0.053, 0.073), 6:(470, 0.053, 0.073), 7:(0, 0,0), \
                8:(43, 0.053, 0.073),9:(86,0.053, 0.073),10:(128,0,0),11:(171,0.053, 0.073)}
    
    rays_2 ={0: (230,0.11,0.13), 1: (256,0.04,0.072), 2: (282,0.11,0.13), 3:(302,0.093,0.113), 4:(302,0.055,0.075), 5:(340,0.054,0.074), 6:(428,0.054,0.074),\
            7:(466,0.055,0.075), 8:(466,0.093,0.113), 9:(486,0.11,0.13), 10:(0,0.052,0.072), 11:(26,0.11,0.13), 12:(46,0.093,0.113), 13:(46,0.055,0.075),\
            14:(84,0.054,0.074), 15:(172,0.054,0.074), 16:(210,0.055,0.075), 17:(210,0.093,0.113)}
    
    rays_3 ={0: (174,0.08, 0.2), 1: (174,0.003, 0.07), 2: (230,0.003, 0.055), 3:(282,0.003, 0.065), 4:(338,0.003, 0.07), 5:(338,0.08, 0.34), 6:(358,0.08, 0.15),\
            7:(384,0.05, 0.1), 8:(410,0.08, 0.15), 9:(430,0.08, 0.15), 10:(430,0.003, 0.07), 11:(486,0.003, 0.065), 12:(26,0.003, 0.065), 13:(44,0.003, 0.07),\
            14:(44,0.08, 0.35), 15:(102,0.08, 0.15), 16:(128,0.05, 0.1), 17:(158,0.08, 0.15)}
    

    
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
    
    def ver_walls(self, rotation):
        shift = self.rotToLidar(rotation)
        # gira los rayos para que estén en orientación Norte
        rangeLocal = self.rangeImage[shift:] + self.rangeImage[:shift]
        walls = {}
        for i in self.rays_2.keys():
            walls[i]=(rangeLocal[self.rays_2[i][0]])
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
        
        # TODO(Zoe): Actualizar las tiles vecinas con la info de las paredes
        # que se comparten con la tile actual
    
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

    def rotToLidar(self, rot):
        #Cuánto girar los rayos para tomar referencia norte
        return int((256/PI)*rot)%512
    
    def hayAlgoIzquierda(self):
        leftDist = self.rangeImage[128]
        return leftDist < 0.08

    def hayAlgoDerecha(self):
        rightDist = self.rangeImage[128*3]
        return rightDist < 0.08
