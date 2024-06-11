from point import Point
from controller import Robot as WebotsRobot
TIME_STEP = 16
class Mapa:
    def __init__(self, robot):
        self.robot = WebotsRobot()
        self.robot = robot
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

    
    def updateVars(self):
        self.updatePosition()

    def updatePosition(self):
        x, _, y = self.gps.getValues()
        self.position = Point(x, y)

    def evaluar_baldosa_delantera(self):
        coordenada_y = self.robot.position['y']
        baldosa_delantera_a_evaluar = coordenada_y - 0.12
        return baldosa_delantera_a_evaluar

    def evaluar_baldosa_derecha(self):
        coordenada_x = self.robot.position['x']
        baldosa_derecha_a_evaluar = coordenada_x + 0.12
        return baldosa_derecha_a_evaluar

    def evaluar_baldosa_izquierda(self):
        coordenada_x = self.position['x']
        baldosa_izquierda_a_evaluar = coordenada_x - 0.12
        return baldosa_izquierda_a_evaluar

    def isVisited(self, baldosas_recorridas, posicion_inicial):
        gridIndex = self.positionToGrid(posicion_inicial)
        if not gridIndex in baldosas_recorridas:
            baldosas_recorridas.append(gridIndex)
            return baldosas_recorridas
        return True

    # def positionToGrid(self, posicion_inicial):
    #     grilla = []
    #     columna = round((self.position.x - posicion_inicial['x']) / 0.12)
    #     grilla.append(columna)
    #     fila = round((self.position.y - posicion_inicial['y']) / 0.12) 
    #     grilla.append(fila)
    #     tupla_grilla = tuple(grilla)
    #     return tupla_grilla
    # def isVisited(self, visitedTiles):
    #     gridIndex = self.positionToGrid()
    #     return gridIndex in visitedTiles

    def positionToGrid(self, posicion_inicial):
        grilla = []
        columna = round((self.position['x'] - posicion_inicial['x']) / 0.12)
        grilla.append(columna)
        fila = round((self.position['y'] - posicion_inicial['y']) / 0.12)
        grilla.append(fila)
        tupla_grilla = tuple(grilla)
        return tupla_grilla
    




