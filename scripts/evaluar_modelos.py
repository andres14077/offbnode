#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-
carpeta_principal = "~/catkin_ws/src/offbnode/"
# carpeta_principal = "~/offbnode/"
import tensorflow as tf
import pandas as pd
import numpy as np
from numpy import expand_dims
from sklearn.preprocessing import LabelEncoder
import os
import time
import matplotlib.pyplot as plt
from joblib import load
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report, ConfusionMatrixDisplay
# from imblearn.metrics import specificity_score
import seaborn as sns

gpu_devices = tf.config.experimental.list_physical_devices('GPU')
for device in gpu_devices:
    tf.config.experimental.set_memory_growth(device, True)


def remove_suffix(input_string, suffix):
    if suffix and input_string.endswith(suffix):
        return input_string[:-len(suffix)]
    return input_string

data = pd.read_csv(carpeta_principal + 'database.csv')
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
carpeta = os.path.expanduser(carpeta_principal + 'neural_model/')
# Filtrar los archivos que terminen con .h5
archivos_h5 = [f for f in os.listdir(carpeta) if os.path.isfile(os.path.join(carpeta, f)) and f.endswith('.h5')]
archivos_h5.sort(reverse=True)

archivos_joblib = [f for f in os.listdir(carpeta) if os.path.isfile(os.path.join(carpeta, f)) and f.endswith('.joblib')]

tiempos = []
precisiones = []
tamaños = []
nombres_archivos=[]
archivos_h5=["ANN_8.h5",
"ANN_16.h5",
"ANN_32.h5",
"ANN_64.h5",
"ANN_128.h5",
"ANN2_8.h5",
"ANN2_16.h5",
"ANN2_32.h5",
"ANN2_64.h5",
"ANN2_128.h5",
"1D_CNN_8.h5",
"1D_CNN_16.h5",
"1D_CNN_32.h5",
"1D_CNN_64.h5",
"1D_CNN_128.h5"]
acuraccy=[66.53,82.66,90.40,93.60,95.86,80.26,87.33,93.73,96.53,99.20,94.40,97.20,98.00,98.93,99.20]
j=0
# for i in archivos_h5:
#     print(i)
#     ruta_modelo = carpeta + i
#     modelo = tf.keras.models.load_model(ruta_modelo)
#     try:
#         test_loss, test_acc = modelo.evaluate(X_tensor, Y_tensor)
#     except ValueError:
#         test_loss, test_acc = modelo.evaluate(X2_tensor, Y_tensor)
#     tamaño_archivo = modelo.count_params()

#     # Ejecuta tu red neuronal aquí (por ejemplo, haciendo predicciones)
#     try:
#         inicio = time.time()
#         predicciones = modelo(X_test_tensor)
#     except ValueError:
#         inicio = time.time()
#         predicciones = modelo(X2_test_tensor)
#     try:
#         inicio = time.time()
#         predicciones = modelo(X_test_tensor)
#     except ValueError:
#         inicio = time.time()
#         predicciones = modelo(X2_test_tensor)
#     fin = time.time()

#     tiempo_ejecucion = fin - inicio

#     if (test_acc>0.2):
#         tiempos.append(tiempo_ejecucion)
#         precisiones.append(test_acc)
#         tamaños.append(tamaño_archivo)
#         nombres_archivos.append(remove_suffix(i,'.h5')+ " %3.2f%%" % (acuraccy[j]) )
#         j+=1

archivos_joblib=["svm_linear.joblib",
"svm_poly.joblib",
"svm_rbf.joblib",
"svm_sigmoid.joblib",
"random_forest.joblib",
"naive_bayes.joblib",
"knn_3.joblib",
"knn_4.joblib",
"knn_8.joblib",
"knn_16.joblib"]
acuraccy=[56.00,100.0,100.0,32.0,99.547,88.000,100.0,96.0,96.0,92.0]

for i in archivos_joblib:
    print(i)
    ruta_modelo = carpeta + i
    modelo_cargado = load(ruta_modelo)
    y_pred_svm = modelo_cargado.predict(X)
    test_acc = accuracy_score(y_encoded, y_pred_svm)
    tamaño_archivo = os.path.getsize(ruta_modelo)

    inicio = time.time()
    # Ejecuta tu red neuronal aquí (por ejemplo, haciendo predicciones)
    predicciones = modelo_cargado.predict(np.reshape(X[0],(1, 25)))

    inicio = time.time()
    # Ejecuta tu red neuronal aquí (por ejemplo, haciendo predicciones)
    predicciones = modelo_cargado.predict(np.reshape(X[0],(1, 25)))
    fin = time.time()

    tiempo_ejecucion = fin - inicio

    if (test_acc>0.2):
        tiempos.append(tiempo_ejecucion)
        precisiones.append(test_acc)
        tamaños.append(tamaño_archivo)
        nombres_archivos.append(remove_suffix(i,'.joblib') + " %3.2f%%" % (acuraccy[j]))

print("Modelo\tTiempo\tPrecision\tTamaño")
for i in range(len(tiempos)):
    print("%s\t%3.2f us\t%3.2f%%\t%d" %(remove_suffix(archivos_joblib[i],'.joblib'),tiempos[i]*1000000,acuraccy[i],tamaños[i]))
plt.figure(figsize=(12,7))
colores = plt.cm.plasma(np.linspace(0, 1, len(tiempos)))
for i in range(len(tiempos)):
    plt.scatter(tiempos[i], tamaños[i], color=colores[i], alpha=0.9,edgecolor='black')

# Anotar cada punto con el nombre del archivo
plt.yscale("log")
plt.xscale("log")
# for i, label in enumerate(nombres_archivos):
#     plt.annotate(label, (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(0,10), ha='center')
i=0
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(5,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(10,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(10,0), ha='left')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(0,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(-20,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(0,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(0,10), ha='center')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(10,0), ha='left')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(-10,0), ha='right')
i+=1
plt.annotate(nombres_archivos[i], (tiempos[i], tamaños[i]), textcoords="offset points", xytext=(0,-15), ha='center')
i+=1


plt.grid(True, which="both")


plt.xlabel('Tiempo de ejecucion (seg)')
plt.ylabel('Numero de parametros')
plt.title('Numero de parametros vs Tiempo de ejecucion')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5), title='Modelos')  # Ubica la leyenda a la derecha de la gráfica
plt.tight_layout()
plt.savefig( "comparacion_modelo_tradicional.png" , dpi=600)
plt.show()

# # Modelo seleccionado
# modelo = tf.keras.models.load_model(carpeta+ 'ANN2_64.h5')
# # Realizar predicciones
# y_pred = modelo.predict(X_tensor)
# y_pred_classes = tf.argmax(y_pred, axis=1).numpy()
# # Generar la matriz de confusión
# matriz_confusion = confusion_matrix(y_encoded, y_pred_classes,normalize='true')
# print("Matriz de Confusión:")
# print(matriz_confusion)
# plt.figure()
# ax = plt.subplot()

# # Usar Seaborn para crear un heatmap
# sns.heatmap(matriz_confusion, annot=True, ax=ax, cmap="RdYlGn", fmt='g')

# # Etiquetas, título y ticks
# ax.set_xlabel('Clases Predichas')
# ax.set_ylabel('Clases Verdaderas')
# ax.set_title('Matriz de Confusión')
# ax.xaxis.set_ticklabels(['Pradera', 'Ladera', 'Valle'])  # Ajustar según tus clases
# ax.yaxis.set_ticklabels(['Pradera', 'Ladera', 'Valle'])

# plt.show()

# # Calcular otras métricas de evaluación
# reporte = classification_report(y_encoded, y_pred_classes,target_names=['Pradera', 'Ladera', 'Valle'])
# print("Reporte de Clasificación:")
# print(reporte)

# especificidad = specificity_score(y_encoded, y_pred_classes, average=None)  # 'None' para obtener resultados por clase

# print("Especificidad por clase:", especificidad)
