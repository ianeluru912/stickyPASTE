nro = 3 # numero de baldosa visitada/guardada
numeros = {} #diccionario de baldosas guardadas: 'baldosa a: pos x, pos y'
numeros['baldosa', nro] = 'tres'

for (key1, key2), value in numeros.items(): #cómo se imprimiría en pantalla o cómo se guardaría
    print(f'{key1} {key2}: {value}')

print(numeros)
nro += 1
numeros['baldosa', nro] = 'cuatro'
print(numeros)