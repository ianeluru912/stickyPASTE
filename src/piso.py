class Piso:
    def __init__(self, r, g, b):
        self.redValue = r
        self.greenValue = g
        self.blueValue = b
    def pantano(self):
        return abs(self.redValue - 169) < 15 \
            and abs(self.greenValue - 135) < 15 \
            and abs(self.blueValue - 75) < 15
    def blackHole(self):
        return abs(self.redValue) < 35 \
            and abs(self.greenValue) < 35 \
            and abs(self.blueValue) < 35
    def green(self):
        return abs(self.redValue - 25) < 15 \
            and abs(self.greenValue - 227) < 15 \
            and abs(self.blueValue - 25) < 15
    def yellow(self):
        return abs(self.redValue - 234) < 15 \
            and abs(self.greenValue - 234) < 15 \
            and abs(self.blueValue - 47) < 15
    def red(self):
        return abs(self.redValue - 234) < 15 \
            and abs(self.greenValue - 47) < 15 \
            and abs(self.blueValue - 47) < 15

    def blue(self):
        return abs(self.redValue - 47) <10 \
            and abs(self.greenValue - 47) < 10 \
            and abs(self.blueValue - 234) < 10

    def purple(self):
        return abs(self.redValue - 109) < 15 \
            and abs(self.greenValue - 47) < 15 \
            and abs(self.blueValue - 188) < 15

    def estandar(self):
        return abs(self.redValue - 195) < 2 \
            and abs(self.greenValue - 195) < 2 \
            and abs(self.blueValue - 195) < 2

    def checkpoint(self):
        # return abs(self.redValue - 70) < 15 \
        #     and abs(self.greenValue - 75) < 15 \
        #     and abs(self.blueValue - 90) < 15
        # return (abs(self.redValue - 42) < 20 \
        #     and abs(self.greenValue - 47) < 20 \
        #     and abs(self.blueValue - 64) < 20) or (abs(self.redValue - 73) < 5 \
        #                                            and abs(self.greenValue - 78) < 5 \
        #                                             and abs(self.blueValue - 94) < 5)
        difRedGreen=abs(self.redValue-self.greenValue)
        difRedBlue=abs(self.redValue-self.blueValue)
        if abs(difRedGreen-5)<4 and (difRedBlue-21)<4:
            return True
    
    def orange(self):
        return abs(self.redValue - 234) < 15 \
            and abs(self.greenValue - 188) < 15 \
            and abs(self.blueValue - 47) < 15