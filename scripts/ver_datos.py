#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

# define el nombre del archivo CSV
# archivo = '/home/andres/offbnode/database.csv'
archivo = '/home/andres/catkin_ws/src/offbnode/database.csv'

# carga el archivo CSV en un DataFrame de Pandas
df = pd.read_csv(archivo)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

Z=df.iloc[int(sys.argv[1]) ]
Z=Z[0:-1]
Z=100-Z
p = np.linspace(0, 2*np.pi, len(Z))
X = 5*np.cos(p)
Y = 5*np.sin(p)

x = np.array([X])
y = np.array([Y])
z = np.array([Z])

ax.plot_wireframe(x, y, z)
ax.set_zlim([0, 100])
plt.show()