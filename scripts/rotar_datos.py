#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import csv
import sys

archivo = '/home/andres/offbnode/database.csv'

# carga el archivo CSV en un DataFrame de Pandas
df = pd.read_csv(archivo)
Z=df.iloc[int(sys.argv[1]) ].to_list()
for i in range(24):
    Zn=[]
    for i in range(1,len(Z)-1):
        Zn.append(Z[i])
    Zn.append(Z[0])
    Zn.append(Z[-1])
    Z=Zn
    # archivo = '/home/andres/offbnode/database2.csv'
    with open(archivo, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(Zn)  # escribe la nueva línea al final del archivo CSV