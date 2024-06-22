class Piso:
    def __init__(self, r, g, b):
        self.red = r
        self.green = g
        self.blue = b
    def pantano(self):
        return abs(self.red - 169) < 15 \
            and abs(self.green - 135) < 15 \
            and abs(self.blue - 75) < 15
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
        return abs(self.red - 47) < 15 \
            and abs(self.green - 47) < 15 \
            and abs(self.blue - 240) < 15

    def violeta(self):
        return abs(self.red - 109) < 15 \
            and abs(self.green - 47) < 15 \
            and abs(self.blue - 188) < 15

    def del_suelo(self):
        return abs(self.red - 195) < 2 \
            and abs(self.green - 195) < 2 \
            and abs(self.blue - 195) < 2

    def checkpoint(self):
        return abs(self.red - 70) < 15 \
            and abs(self.green - 75) < 15 \
            and abs(self.blue - 90) < 15
