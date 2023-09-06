#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from tensorflow.keras.callbacks import ModelCheckpoint
from sklearn.metrics import confusion_matrix
import seaborn as sns
import matplotlib.pyplot as plt
from numpy import mean
from numpy import std
from numpy import expand_dims
import os
from keras.utils.vis_utils import plot_model

def entrenar_modelo_1(trainX, trainy, testX, testy,N_redes ,r ):
    # Construir el modelo de la red neuronal
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(N_redes, activation='relu', input_shape=(trainX.shape[1],)),
        tf.keras.layers.Dense(3, activation='softmax')
    ])

    # Compilar el modelo
    optimizer = tf.keras.optimizers.Adam(learning_rate=0.0001)
    plot_model(model, show_shapes=True, to_file='modelo_1_N_' + str(N_redes) + '.png')
    model.compile(optimizer=optimizer, loss='sparse_categorical_crossentropy', metrics=['accuracy'])

    # Definir el callback para guardar el mejor modelo según la precisión en el conjunto de prueba
    checkpoint_callback = ModelCheckpoint('mejor_modelo_1_N_' + str(N_redes) + '.h5', monitor='val_accuracy', save_best_only=True, mode='max', verbose=1)

    # Entrenar el modelo con el callback
    model.fit(trainX, trainy, epochs=200, batch_size=3, validation_data=(testX, testy), callbacks=[checkpoint_callback])

    # Cargar el mejor modelo guardado
    best_model = tf.keras.models.load_model('mejor_modelo_1_N_' + str(N_redes) + '.h5')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_loss, test_acc = best_model.evaluate(testX, testy)
    os.system('mv mejor_modelo_1_N_' + str(N_redes) + '.h5 mejor_modelo_1_N_' + str(N_redes) + '_I_' + str(r) +'_A_' + str(test_acc) + '.h5')
    print('Best model test accuracy:', test_acc)
    return test_acc

def entrenar_modelo_2(trainX, trainy, testX, testy , N , r ):
    # Construir el modelo de la red neuronal
    print(trainX.shape[1])
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(N, activation='relu', input_shape=(trainX.shape[1],)),
        tf.keras.layers.Dropout(0.5),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(100, activation='relu'),
        tf.keras.layers.Dense(3, activation='softmax')
    ])

    # Compilar el modelo
    optimizer = tf.keras.optimizers.Adam(learning_rate=0.0001)
    plot_model(model, show_shapes=True, to_file='modelo_2_N_' + str(N) + '.png')
    model.compile(optimizer=optimizer, loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    print(model.summary())
    # Definir el callback para guardar el mejor modelo según la precisión en el conjunto de prueba
    checkpoint_callback = ModelCheckpoint('mejor_modelo_2_N_' + str(N) + '.h5', monitor='val_accuracy', save_best_only=True, mode='max', verbose=1)

    # Entrenar el modelo con el callback
    model.fit(trainX, trainy, epochs=200, batch_size=3, validation_data=(testX, testy), callbacks=[checkpoint_callback])

    # Cargar el mejor modelo guardado
    best_model = tf.keras.models.load_model('mejor_modelo_2_N_' + str(N) + '.h5')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_loss, test_acc = best_model.evaluate(testX, testy)
    os.system('mv mejor_modelo_2_N_' + str(N) + '.h5 mejor_modelo_2_N_' + str(N) + '_I_' + str(r) + '_A_' + str(test_acc) + '.h5')
    print('Best model test accuracy:', test_acc)
    return test_acc

def entrenar_modelo_3(trainX, trainy, testX, testy , N , r ):
    # Construir el modelo de la red neuronal
    model = tf.keras.Sequential([
        tf.keras.layers.Conv1D(filters=N, kernel_size=3, activation='relu', input_shape=(trainX.shape[1],1,)),
        tf.keras.layers.Conv1D(filters=N, kernel_size=3, activation='relu'),
        tf.keras.layers.Dropout(0.5),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(100, activation='relu'),
        tf.keras.layers.Dense(3, activation='softmax')
    ])

    # Compilar el modelo
    optimizer = tf.keras.optimizers.Adam(learning_rate=0.0001)
    plot_model(model, show_shapes=True, to_file='modelo_3_N_' + str(N) + '.png')
    model.compile(optimizer=optimizer, loss='sparse_categorical_crossentropy', metrics=['accuracy'])

    # Definir el callback para guardar el mejor modelo según la precisión en el conjunto de prueba
    checkpoint_callback = ModelCheckpoint('mejor_modelo_3_N_' + str(N) + '.h5', monitor='val_accuracy', save_best_only=True, mode='max', verbose=1)

    # Entrenar el modelo con el callback
    model.fit(trainX, trainy, epochs=200, batch_size=3, validation_data=(testX, testy), callbacks=[checkpoint_callback])

    # Cargar el mejor modelo guardado
    best_model = tf.keras.models.load_model('mejor_modelo_3_N_' + str(N) + '.h5')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_loss, test_acc = best_model.evaluate(testX, testy)
    os.system('mv mejor_modelo_3_N_' + str(N) + '.h5 mejor_modelo_3_N_' + str(N) + '_I_' + str(r) + '_A_' + str(test_acc) + '.h5')
    print('Best model test accuracy:', test_acc)
    return test_acc

# summarize scores
def summarize_results(scores, params,patch):
    print(scores, params)
    # summarize mean and standard deviation
    for i in range(len(scores)):
        m, s = mean(scores[i]), std(scores[i])
        print('Param=%s: %.3f%% (+/-%.3f)' % (params[i], m, s))
    # boxplot of scores
    plt.boxplot(scores, labels=params)
    plt.xticks(rotation='vertical')
    plt.subplots_adjust(top=0.9)
    plt.subplots_adjust(bottom=0.25)
    plt.savefig(patch)
    plt.show()
# run an experiment
def run_experiment(trainX, trainy, testX, testy, trainX_2, testX_2, repeats=10):
    # repeat experiment

    all_scores = list()
    tamaño_red=[8,16,32,64,128,256,512,1024]

    all_scores=[[69.33333277702332, 69.33333277702332, 68.00000071525574, 78.66666913032532, 60.00000238418579, 72.00000286102295, 72.00000286102295, 61.33333444595337, 53.33333611488342, 61.33333444595337], [82.66666531562805, 69.33333277702332, 93.33333373069763, 81.33333325386047, 83.99999737739563, 85.33333539962769, 81.33333325386047, 80.0000011920929, 78.66666913032532, 90.66666960716248], [92.00000166893005, 92.00000166893005, 87.99999952316284, 94.66666579246521, 95.99999785423279, 89.33333158493042, 87.99999952316284, 86.66666746139526, 90.66666960716248, 86.66666746139526], [93.33333373069763, 90.66666960716248, 94.66666579246521, 93.33333373069763, 94.66666579246521, 86.66666746139526, 95.99999785423279, 94.66666579246521, 95.99999785423279, 95.99999785423279], [95.99999785423279, 94.66666579246521, 98.66666793823242, 95.99999785423279, 93.33333373069763, 94.66666579246521, 95.99999785423279, 97.33333587646484, 94.66666579246521, 97.33333587646484], [94.66666579246521, 98.66666793823242, 98.66666793823242, 100.0, 100.0, 98.66666793823242, 100.0, 97.33333587646484, 97.33333587646484, 98.66666793823242], [100.0, 98.66666793823242, 100.0, 100.0, 97.33333587646484, 100.0, 100.0, 98.66666793823242, 100.0, 100.0], [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], [74.6666669845581, 87.99999952316284, 73.33333492279053, 74.6666669845581, 87.99999952316284, 82.66666531562805, 75.99999904632568, 85.33333539962769, 81.33333325386047, 78.66666913032532], [87.99999952316284, 85.33333539962769, 86.66666746139526, 94.66666579246521, 83.99999737739563, 83.99999737739563, 77.33333110809326, 90.66666960716248, 89.33333158493042, 93.33333373069763], [95.99999785423279, 97.33333587646484, 89.33333158493042, 95.99999785423279, 92.00000166893005, 92.00000166893005, 94.66666579246521, 97.33333587646484, 92.00000166893005, 90.66666960716248], [94.66666579246521, 97.33333587646484, 97.33333587646484, 97.33333587646484, 95.99999785423279, 98.66666793823242, 94.66666579246521, 98.66666793823242, 95.99999785423279, 94.66666579246521], [100.0, 100.0, 100.0, 97.33333587646484, 98.66666793823242, 100.0, 98.66666793823242, 100.0, 100.0, 97.33333587646484], [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], [92.00000166893005, 98.66666793823242, 93.33333373069763, 95.99999785423279, 95.99999785423279, 95.99999785423279, 97.33333587646484, 92.00000166893005, 92.00000166893005, 90.66666960716248], [97.33333587646484, 100.0, 95.99999785423279, 93.33333373069763, 98.66666793823242, 98.66666793823242, 97.33333587646484, 100.0, 95.99999785423279, 94.66666579246521], [100.0, 98.66666793823242, 98.66666793823242, 97.33333587646484, 98.66666793823242, 95.99999785423279, 98.66666793823242, 97.33333587646484, 94.66666579246521, 100.0], [98.66666793823242, 100.0, 98.66666793823242, 98.66666793823242, 98.66666793823242, 97.33333587646484, 98.66666793823242, 98.66666793823242, 100.0, 100.0], [100.0, 98.66666793823242, 100.0, 98.66666793823242, 98.66666793823242, 100.0, 98.66666793823242, 98.66666793823242, 100.0, 98.66666793823242], [100.0, 98.66666793823242, 100.0, 100.0, 100.0, 100.0, 98.66666793823242, 98.66666793823242, 100.0, 98.66666793823242], [98.66666793823242, 100.0, 100.0, 100.0, 100.0, 98.66666793823242, 100.0, 100.0, 98.66666793823242, 98.66666793823242], [98.66666793823242, 98.66666793823242, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 98.66666793823242, 100.0]]
    # for N in tamaño_red:
    #     scores = list()
    #     for r in range(repeats):
    #         score = entrenar_modelo_1(trainX, trainy, testX, testy,N,r)
    #         score = score * 100.0
    #         print('>p=%s #%d: %.3f' % (N, r+1, score))
    #         scores.append(score)
    #     all_scores.append(scores)

    summarize_results(all_scores[0:8],["ANN-8","ANN-16","ANN-32","ANN-64","ANN-128","ANN-256","ANN-512","ANN-1024"],"desviacion_estandar_modelo_1.png")

    # for N in tamaño_red:
    #     scores = list()
    #     for r in range(repeats):
    #         score = entrenar_modelo_2(trainX, trainy, testX, testy,N,r)
    #         score = score * 100.0
    #         print('>p=%s #%d: %.3f' % (N, r+1, score))
    #         scores.append(score)
    #     all_scores.append(scores)

    summarize_results(all_scores[8:16],["ANN2-8","ANN2-16","ANN2-32","ANN2-64","ANN2-128","ANN2-256","ANN2-512","ANN2-1024"],"desviacion_estandar_modelo_2.png")

    # for N in tamaño_red:
    #     scores = list()
    #     for r in range(repeats):
    #         score = entrenar_modelo_3(trainX_2, trainy, testX_2, testy,N,r)
    #         score = score * 100.0
    #         print('>p=%s #%d: %.3f' % (N, r+1, score))
    #         scores.append(score)
    #     all_scores.append(scores)

    summarize_results(all_scores[16:24],["1D-CNN-8","1D-CNN-16","1D-CNN-32","1D-CNN-64","1D-CNN-128","1D-CNN-256","1D-CNN-512","1D-CNN-1024"],"desviacion_estandar_modelo_3.png")

    # summarize results
    summarize_results(all_scores,["ANN-8","ANN-16","ANN-32","ANN-64","ANN-128","ANN-256","ANN-512","ANN-1024","ANN2-8","ANN2-16","ANN2-32","ANN2-64","ANN2-128","ANN2-256","ANN2-512","ANN2-1024","1D-CNN-8","1D-CNN-16","1D-CNN-32","1D-CNN-64","1D-CNN-128","1D-CNN-256","1D-CNN-512","1D-CNN-1024"],"desviacion_estandar_total.png")

# Cargar datos desde un archivo CSV
data = pd.read_csv('~/offbnode/database.csv')
# data = pd.read_csv('~/catkin_ws/src/offbnode/database.csv')
# Dividir los datos en características (X) y etiquetas (y)
X = data.drop('clasificacion', axis=1)  # Ajusta 'etiqueta' al nombre de la columna que contiene tus etiquetas
# X = expand_dims(X.values, axis=2)
y = data['clasificacion']

# Codificar las etiquetas como valores numéricos
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
class_labels = label_encoder.classes_  # Obtener las etiquetas de clases originales

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)

X_train_2=expand_dims(X_train.values, axis=2)
X_test_2=expand_dims(X_test.values, axis=2)

# Convertir los datos a tensores de TensorFlow
X_train_tensor = tf.convert_to_tensor(X_train, dtype=tf.float32)
X_test_tensor = tf.convert_to_tensor(X_test, dtype=tf.float32)
X_train_2_tensor = tf.convert_to_tensor(X_train_2, dtype=tf.float32)
X_test_2_tensor = tf.convert_to_tensor(X_test_2, dtype=tf.float32)
y_train_tensor = tf.convert_to_tensor(y_train, dtype=tf.int32)
y_test_tensor = tf.convert_to_tensor(y_test, dtype=tf.int32)

run_experiment(X_train_tensor , y_train_tensor , X_test_tensor , y_test_tensor, X_train_2_tensor, X_test_2_tensor)

# # Realizar predicciones con el modelo
# y_pred = best_model.predict(X_test_tensor)
# y_pred_classes = tf.argmax(y_pred, axis=1).numpy()
# # Crear matriz de confusión
# cm = confusion_matrix(y_test, y_pred_classes)

# # Mostrar matriz de confusión
# sns.heatmap(cm, annot=True, fmt='d', cmap='Blues')
# plt.xlabel('Predicted')
# plt.ylabel('Actual')
# plt.xticks(ticks=range(len(class_labels)), labels=class_labels, rotation=45)
# plt.yticks(ticks=range(len(class_labels)), labels=class_labels, rotation=0)
# plt.show()