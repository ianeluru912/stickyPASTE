
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