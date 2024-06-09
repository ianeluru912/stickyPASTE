from robot import Robot

class Mapa:
    def __init__(self):
        self.robot = Robot()
        
    def evaluar_baldosa_delantera(self):
        coordenada_y = self.robot.position['y']
        baldosa_delantera_a_evaluar = coordenada_y - 0.12
        return baldosa_delantera_a_evaluar

    def evaluar_baldosa_derecha(self):
        coordenada_x = self.robot.position['x']
        baldosa_derecha_a_evaluar = coordenada_x + 0.12
        return baldosa_derecha_a_evaluar

    def evaluar_baldosa_izquierda(self):
        coordenada_x = self.robot.position['x']
        baldosa_izquierda_a_evaluar = coordenada_x - 0.12
        return baldosa_izquierda_a_evaluar

    def isVisited(self, visitedTiles):
        gridIndex = self.positionToGrid()
        return gridIndex in visitedTiles

    def positionToGrid(self, posicion_inicial):
        grilla = []
        columna = round((self.robot.position['x'] - posicion_inicial['x']) / 0.12)
        grilla.append(columna)
        fila = round((self.robot.position['y'] - posicion_inicial['y']) / 0.12)
        grilla.append(fila)
        tupla_grilla = tuple(grilla)
        return tupla_grilla