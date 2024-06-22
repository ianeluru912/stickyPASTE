
class Lidar:

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