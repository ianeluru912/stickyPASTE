fila1 = {'x': 0.30030265846590987, 'y': -0.0593942202450804}
posiciones = []
posiciones.append(fila1)
fila2 = {'x': -0.30030265846590987, 'y': -0.0593942202450804}
posiciones.append(fila2)
posiciones.sort(key=lambda posicion: posicion['x'])
print(posiciones)
