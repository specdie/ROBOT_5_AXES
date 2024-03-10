import numpy as np

# Paramètres DH
d1 = 10,5
a2 = 11,3
a3 = 11
a4 = 5

# Valeurs des articulations (en radians)
theta1 = 0.5
theta2 = 0.8
theta3 = -0.4
theta4 = 1.2
theta5 = -0.7

# Matrices de transformation homogène pour chaque articulation
A1 = np.array([
    [np.cos(theta1), -np.sin(theta1), 0, 0],
    [np.sin(theta1), np.cos(theta1), 0, 0],
    [0, 0, 1, d1],
    [0, 0, 0, 1]
])

A2 = np.array([
    [np.cos(theta2), -np.sin(theta2), 0, a2],
    [0, 0, -1, 0],
    [np.sin(theta2), np.cos(theta2), 0, 0],
    [0, 0, 0, 1]
])

A3 = np.array([
    [np.cos(theta3), -np.sin(theta3), 0, a3],
    [0, 0, 1, 0],
    [-np.sin(theta3), -np.cos(theta3), 0, 0],
    [0, 0, 0, 1]
])

A4 = np.array([
    [np.cos(theta4), -np.sin(theta4), 0, a4],
    [0, 0, -1, 0],
    [np.sin(theta4), np.cos(theta4), 0, 0],
    [0, 0, 0, 1]
])

A5 = np.array([
    [np.cos(theta5), 0, np.sin(theta5), 0],
    [0, 1, 0, 0],
    [-np.sin(theta5), 0, np.cos(theta5), 0],
    [0, 0, 0, 1]
])

# Matrice totale du MGD
T_total = np.dot(np.dot(np.dot(np.dot(A1, A2), A3), A4), A5)

# Coordonnées du point final
x = T_total[0, 3]
y = T_total[1, 3]
z = T_total[2, 3]

print("Coordonnées du point final :")
print(f"x = {x} cm")
print(f"y = {y} cm")
print(f"z = {z} cm")
