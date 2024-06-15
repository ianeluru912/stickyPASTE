import math
def normalizar_radianes(radianes): # radianes seria la rotacion actual del robot
    if radianes > math.pi:
        radianes -= math.pi*2
        return radianes
    elif radianes < -math.pi:
        radianes += math.pi*2
        return radianes
    return radianes

n1 = 4.7158009691060885 # angulo primer giro d.
n2 = 3.148262702429781 # angulo segundo giro d.
# print(n1 -n2)

n3 = 6.283185306660025 # angulo 0
n4 = 4.7158009691060885 # angulo luego del primer giro d.
# print(n3 - n4)

n5 = 3.148262702429781 # angulo segundo giro d.
n6 = 1.5808212914718471 # angulo tercer giro d.
# print(n5 - n6)

n7 = 3.148238415305075 # angulo primer giro i.
n8 = 1.5808212914718471 # angulo tercer giro d.
# print(n7 - n8)

print(normalizar_radianes(n5))