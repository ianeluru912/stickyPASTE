def agregar_numeros(numeros):
    numero = 3
    numeros.append(numero)
    return numeros
numeros = []
for i in range(3):
    print(agregar_numeros(numeros))