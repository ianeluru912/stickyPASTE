class Piso:
    def __init__(self, r, g, b):
        self.red = r
        self.green = g
        self.blue = b
    def baldosa(self):
        return abs(self.red - 249) < 15 \
            and abs(self.green) < 15 \
            and abs(self.blue) < 15
    def pantano(self):
        return abs(self.red - 152) < 15 \
            and abs(self.green - 119) < 15 \
            and abs(self.blue - 60) < 15
    def blackHole(self):
        return abs(self.red) < 15 \
            and abs(self.green) < 15 \
            and abs(self.blue) < 15
    