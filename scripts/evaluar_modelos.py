#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-

import tensorflow as tf
import pandas as pd
import numpy as np
from numpy import expand_dims
from sklearn.preprocessing import LabelEncoder
import os
import time
import matplotlib.pyplot as plt
from joblib import load
from sklearn.metrics import accuracy_score


gpu_devices = tf.config.experimental.list_physical_devices('GPU')
for device in gpu_devices:
    tf.config.experimental.set_memory_growth(device, True)


def remove_suffix(input_string, suffix):
    if suffix and input_string.endswith(suffix):
        return input_string[:-len(suffix)]
    return input_string

# data = pd.read_csv('~/offbnode/database.csv')
data = pd.read_csv('~/catkin_ws/src/offbnode/database.csv')
# Dividir los datos en características (X) y etiquetas (y)
X = data.drop('clasificacion', axis=1)  # Ajusta 'etiqueta' al nombre de la columna que contiene tus etiquetas
# X = expand_dims(X.values, axis=2)
y = data['clasificacion']

# Codificar las etiquetas como valores numéricos
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
class_labels = label_encoder.classes_  # Obtener las etiquetas de clases originales

X_tensor = tf.convert_to_tensor(X, dtype=tf.float32)
X2_tensor = tf.convert_to_tensor(expand_dims(X.values, axis=2), dtype=tf.float32)
Y_tensor = tf.convert_to_tensor(y_encoded, dtype=tf.float32)
X_test_tensor = tf.convert_to_tensor(np.reshape(X.iloc[0].tolist(),(1, 25)), dtype=tf.float32)
X2_test_tensor = tf.convert_to_tensor(expand_dims(np.reshape(X.iloc[0].tolist(),(1, 25)), axis=2), dtype=tf.float32)

X = np.array(X, order='C')
y_encoded = np.array(y_encoded, order='C')


# Cargar el modelo
# carpeta = os.path.expanduser('~/offbnode/neural_model/')
carpeta = os.path.expanduser('~/catkin_ws/src/offbnode/neural_model/')
# Filtrar los archivos que terminen con .h5
archivos_h5 = [f for f in os.listdir(carpeta) if os.path.isfile(os.path.join(carpeta, f)) and f.endswith('.h5')]

archivos_joblib = [f for f in os.listdir(carpeta) if os.path.isfile(os.path.join(carpeta, f)) and f.endswith('.joblib')]

tiempos = []
precisiones = []
tamaños = []
nombres_archivos=[]

for i in archivos_h5:
    print(i)
    ruta_modelo = carpeta + i
    modelo = tf.keras.models.load_model(ruta_modelo)
    try:
        test_loss, test_acc = modelo.evaluate(X_tensor, Y_tensor)
    except ValueError:
        test_loss, test_acc = modelo.evaluate(X2_tensor, Y_tensor)
    tamaño_archivo = os.path.getsize(ruta_modelo)

    inicio = time.time()
    # Ejecuta tu red neuronal aquí (por ejemplo, haciendo predicciones)
    try:
        predicciones = modelo(X_test_tensor)
    except ValueError:
        predicciones = modelo(X2_test_tensor)
    fin = time.time()

    tiempo_ejecucion = fin - inicio


    tiempos.append(1/tiempo_ejecucion)
    precisiones.append(test_acc)
    tamaños.append(tamaño_archivo)
    nombres_archivos.append(remove_suffix(i,'.h5'))

for i in archivos_joblib:
    ruta_modelo = carpeta + i
    modelo_cargado = load(ruta_modelo)
    y_pred_svm = modelo_cargado.predict(X)
    test_acc = accuracy_score(y_encoded, y_pred_svm)
    tamaño_archivo = os.path.getsize(ruta_modelo)

    inicio = time.time()
    # Ejecuta tu red neuronal aquí (por ejemplo, haciendo predicciones)
    predicciones = modelo_cargado.predict(np.reshape(X[0],(1, 25)))
    fin = time.time()

    tiempo_ejecucion = fin - inicio


    tiempos.append(1/tiempo_ejecucion)
    precisiones.append(test_acc)
    tamaños.append(tamaño_archivo)
    nombres_archivos.append(remove_suffix(i,'.joblib'))

tamaños_10=[]
for i in tamaños:
    tamaños_10.append(i/1000000)

precisiones_10=[]
for i in precisiones:
    precisiones_10.append(i*100)


colores = plt.cm.plasma(np.linspace(0, 1, len(tiempos)))
for i in range(len(tiempos)):
    plt.scatter(tiempos[i], tamaños_10[i], s=precisiones_10[i], color=colores[i], alpha=0.9,edgecolor='black', label=nombres_archivos[i])

# Crear una leyenda con puntos de tamaño uniforme
legend_elements = [plt.Line2D([0], [0], marker='o', color='w', label=nombres_archivos[i], markersize=10, markerfacecolor=colores[i]) for i in range(len(tiempos))]
plt.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5), title='Modelos')
# Anotar cada punto con el nombre del archivo
plt.yscale("log")
plt.xscale("log")

# for i, nombre in enumerate(nombres_archivos):
#     plt.annotate(nombre, (tiempos[i], precisiones[i]), fontsize=9, alpha=0.7)


plt.xlabel('FPS')
plt.ylabel('Precisión')
plt.title('FPS vs Precisión')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5), title='Modelos')  # Ubica la leyenda a la derecha de la gráfica
plt.tight_layout()
plt.savefig("comparacion.png")
plt.show()