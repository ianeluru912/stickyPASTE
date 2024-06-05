from controller import Robot
import math

TIME_STEP = 32
MAX_VEL = 6.28

robot = Robot()

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

iu = robot.getDevice("inertial_unit")
iu.enable(TIME_STEP)

ruedaI = robot.getDevice("ruedaI motor")
ruedaI.setPosition(float("inf"))
ruedaI.setVelocity(0)

ruedaD = robot.getDevice("ruedaD motor")
ruedaD.setPosition(float("inf"))
ruedaD.setVelocity(0)

di = robot.getDevice("distanciaIzquierda")
di.enable(TIME_STEP)

dd = robot.getDevice("distanciaDerecha")
dd.enable(TIME_STEP)

df = robot.getDevice("distanciaFrente")
df.enable(TIME_STEP)

position = {"x": 0, "y": 0}
rotation = 0

def updatePosition():
    global position
    x, _, y = gps.getValues()
    position = {"x": x, "y": y}

def updateRotation():
    global rotation
    _, _, rotation = iu.getRollPitchYaw()

def updateVars():
    updatePosition()
    updateRotation()

def step():
    result = robot.step(TIME_STEP)
    updateVars()
    return result

def angle_diff(a, b):
    clockwise = (a - b) % math.tau
    counterclockwise = (b - a) % math.tau
    return min(clockwise, counterclockwise)

def girar(rad):
    lastRot = rotation
    deltaRot = 0

    while step() != -1:
        deltaRot += angle_diff(rotation, lastRot)
        lastRot = rotation

        diff = angle_diff(deltaRot, abs(rad))

        if diff <= 0.01:
            break

        mul = (4 / math.pi) * diff
        mul = min(max(mul, 0.1), 1)

        if rad > 0:
            ruedaI.setVelocity(mul * MAX_VEL)
            ruedaD.setVelocity(-mul * MAX_VEL)
        else:
            ruedaI.setVelocity(-mul * MAX_VEL)
            ruedaD.setVelocity(mul * MAX_VEL)

    ruedaI.setVelocity(0)
    ruedaD.setVelocity(0)

def dist(pt1, pt2):
    return math.sqrt((pt2["x"] - pt1["x"]) ** 2 + (pt2["y"] - pt1["y"]) ** 2)

def avanzar(distance):
    initPos = position

    while step() != -1:
        diff = abs(distance) - dist(position, initPos)

        vel = min(max(diff / 0.01, 0.1), 1)
        if distance < 0: vel = -1

        ruedaI.setVelocity(vel * MAX_VEL)
        ruedaD.setVelocity(vel * MAX_VEL)

        if diff < 0.001:
            break

    ruedaI.setVelocity(0)
    ruedaD.setVelocity(0)

def avanzarBien(difurcaciones, baldosas_recorridas, nro_baldosa):
    baldosa_avanzada = 0
    if distF < 0.13:
        if distI < 0.13 and distD < 0.13:
            girar(-0.25 * math.tau)
        elif distI < 0.13:
            girar(0.25 * math.tau)
        elif distD < 0.13:
            girar(-0.25 * math.tau)
        else:
            girar(-0.25 * math.tau)
    else:
        avanzar(0.12)
        # print(difurcaciones)
        nro_baldosa += 1
        baldosas_recorridas['baldosa', nro_baldosa] = position.copy() 
        for (key1, key2), value in baldosas_recorridas.items():
            print(f'{key1} {key2}: {value}')
        baldosa_avanzada = 1
        if distI >= 0.13 or distD >= 0.13 and distF <= 0.08:
            if baldosa_avanzada == 1 and len(difurcaciones) == 0:
                print('primera difurcacion')
                difurcaciones.append(position.copy())
                print(difurcaciones)
        elif distF >= 0.13 and (distI >= 0.13 or distD >= 0.13):
            if baldosa_avanzada == 1 and len(difurcaciones) == 0:
                print('primera difurcacion')
                difurcaciones.append(position.copy())
                print(difurcaciones)
    return nro_baldosa

def evaluar_filas_columnas(filas, columnas):
    posicion_a_evaluar = position.copy() 
    if position['y'] - posicion_a_evaluar['y'] <= 0.06: # adelante
        filas -= 1
        return filas
    elif position['x'] - posicion_a_evaluar['x'] >= 0.06: # derecha
        columnas += 1
        return columnas
    elif position['y'] - posicion_a_evaluar['y'] >= 0.06: # atras
        filas += 1
        return filas
    elif position['x'] - posicion_a_evaluar['x'] <= 0.06: # izquierda
        columnas -= 1
        return columnas


difurcaciones = []
baldosas_recorridas = {}
nro_baldosa = 0
start_time = robot.getTime()
me_aleje_de_difurcacion = False
filas = 0
columnas = 0


step()
intial_position = position.copy()
while step() != -1:
    distI = di.getValue()
    distD = dd.getValue()
    distF = df.getValue()
    nro_baldosa = avanzarBien(difurcaciones, baldosas_recorridas, nro_baldosa)
    if difurcaciones and dist(position, difurcaciones[0]) > 0.5:
        me_aleje_de_difurcacion = True
    if difurcaciones and dist(position, difurcaciones[0]) < 0.1 and me_aleje_de_difurcacion:
        if distF > 0.13 and distD >= 0.13:
            girar(0.25 * math.tau)
        elif distF > 0.13 and distI >= 0.13:
            girar(-0.25 * math.tau)
        me_aleje_de_difurcacion = False
    if (robot.getTime() - start_time) * 1000.0 >= 120000:
        break