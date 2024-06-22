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
        return abs(self.red) < 30 \
            and abs(self.green) < 30 \
            and abs(self.blue) < 30
    def verde(self):
        return abs(self.red - 48) < 15 \
            and abs(self.green - 255) < 15 \
            and abs(self.blue - 48) < 15

    def rojo(self):
        return abs(self.red - 255) < 15 \
            and abs(self.green - 91) < 15 \
            and abs(self.blue - 91) < 15

    def azul(self):
        return abs(self.red - 91) < 15 \
            and abs(self.green - 91) < 15 \
            and abs(self.blue - 255) < 15

    def violeta(self):
        return abs(self.red - 193) < 15 \
            and abs(self.green - 93) < 15 \
            and abs(self.blue - 251) < 15

    def del_suelo(self):
        return abs(self.red - 252) < 2 \
            and abs(self.green - 252) < 2 \
            and abs(self.blue - 252) < 2

    def checkpoint(self):
        return abs(self.red - 255) < 2 \
            and abs(self.green - 255) < 2 \
            and abs(self.blue - 255) < 2
