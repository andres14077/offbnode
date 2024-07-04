#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import confusion_matrix
from sklearn.svm import SVC
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.neighbors import KNeighborsClassifier
from joblib import dump

from sklearn.metrics import accuracy_score
import seaborn as sns
import matplotlib.pyplot as plt
from numpy import mean
from numpy import std
from numpy import expand_dims

def entrenar_modelo_4(trainX, trainy, testX, testy,Kernel ):
    # Construir modelo
    svm_model = SVC(kernel=Kernel)
    svm_model.fit(trainX, trainy)
    y_pred_svm = svm_model.predict(testX)
    dump(svm_model, 'svm_'+Kernel+'.joblib')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_acc = accuracy_score(testy, y_pred_svm)
    print('Best model test accuracy:', test_acc)
    return test_acc

def entrenar_modelo_5(trainX, trainy, testX, testy ):
    # Construir modelo
    random_forest_model = RandomForestClassifier()
    random_forest_model.fit(trainX, trainy)
    y_pred_svm = random_forest_model.predict(testX)
    dump(random_forest_model, 'random_forest.joblib')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_acc = accuracy_score(testy, y_pred_svm)
    print('Best model test accuracy:', test_acc)
    return test_acc

def entrenar_modelo_6(trainX, trainy, testX, testy ):
    # Construir modelo
    naive_bayes_model = GaussianNB()
    naive_bayes_model.fit(trainX, trainy)
    y_pred_svm = naive_bayes_model.predict(testX)
    dump(naive_bayes_model, 'naive_bayes.joblib')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_acc = accuracy_score(testy, y_pred_svm)
    print('Best model test accuracy:', test_acc)
    return test_acc

def entrenar_modelo_7(trainX, trainy, testX, testy , N ):
    # Construir modelo
    knn_model = KNeighborsClassifier(n_neighbors=N)
    knn_model.fit(trainX, trainy)
    y_pred_svm = knn_model.predict(testX)
    dump(knn_model, 'knn_' + str(N) + '.joblib')

    # Evaluar el mejor modelo en el conjunto de prueba
    test_acc = accuracy_score(testy, y_pred_svm)
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
    plt.savefig(patch, dpi=600)
    plt.show()
# run an experiment
def run_experiment(trainX, trainy, testX, testy, repeats=50):
    # repeat experiment

    all_scores = list()
    kernels=["linear", "poly", "rbf", "sigmoid"]

    all_scores=[]
    for N in kernels:
        scores = list()
        for r in range(repeats):
            score = entrenar_modelo_4(trainX, trainy, testX, testy,N)
            score = score * 100.0
            print('>p=%s #%d: %.3f' % (N, r+1, score))
            scores.append(score)
        all_scores.append(scores)

    # summarize_results(all_scores[0:4],["SVM-linear","SVM-poly","SVM-rbf","SVM-sigmoid"],"desviacion_estandar_modelo_4.png")

    scores = list()
    for r in range(repeats):
        score = entrenar_modelo_5(trainX, trainy, testX, testy)
        score = score * 100.0
        print('>p=%s #%d: %.3f' % ("RandomForest", r+1, score))
        scores.append(score)
    all_scores.append(scores)

    # summarize_results(all_scores[4],["RandomForest"],"desviacion_estandar_modelo_5.png")

    scores = list()
    for r in range(repeats):
        score = entrenar_modelo_6(trainX, trainy, testX, testy)
        score = score * 100.0
        print('>p=%s #%d: %.3f' % ("Naive-Bayes", r+1, score))
        scores.append(score)
    all_scores.append(scores)

    # summarize_results(all_scores[5],["Naive-Bayes"],"desviacion_estandar_modelo_6.png")

    vecinos=[3,4,8,16]
    for N in vecinos:
        scores = list()
        for r in range(repeats):
            score = entrenar_modelo_7(trainX, trainy, testX, testy,N)
            score = score * 100.0
            print('>p=%s #%d: %.3f' % (N, r+1, score))
            scores.append(score)
        all_scores.append(scores)



    # summarize results
    summarize_results(all_scores,["SVM-linear","SVM-poly","SVM-rbf","SVM-sigmoid","RandomForest","Naive-Bayes","KNN-3","KNN-4","KNN-8","KNN-16"],"desviacion_estandar_clasicos_total.png")

# Cargar datos desde un archivo CSV
# data = pd.read_csv('~/offbnode/database.csv')
data = pd.read_csv('~/catkin_ws/src/offbnode/database.csv')
# Dividir los datos en características (X) y etiquetas (y)
X = data.drop('clasificacion', axis=1)  # Ajusta 'etiqueta' al nombre de la columna que contiene tus etiquetas
y = data['clasificacion']

# Codificar las etiquetas como valores numéricos
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
class_labels = label_encoder.classes_  # Obtener las etiquetas de clases originales

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)

X_train = np.array(X_train, order='C')
X_test = np.array(X_test, order='C')

run_experiment(X_train , y_train , X_test , y_test )

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