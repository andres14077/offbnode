#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import textwrap
def dibujar_red_neuronal(ax, izquierda, derecha, abajo, arriba, tamanos_de_capa, colores_de_capa, label_de_capa):
    '''
    Dibuja una red neuronal con capas de diferentes colores y puntos suspensivos para capas intermedias.
    tamanos_de_capa: lista de enteros con el número de neuronas en cada capa.
    colores_de_capa: lista de colores para cada capa.
    '''
    espaciado_vertical = (arriba - abajo) / float(max(tamanos_de_capa))
    espaciado_horizontal = (derecha - izquierda) / float(len(tamanos_de_capa) - 1)
    # Nodos
    for n, (tamanio_de_capa, color_de_capa) in enumerate(zip(tamanos_de_capa, colores_de_capa)):
        tope_de_capa = espaciado_vertical * (tamanio_de_capa - 1) / 2. + (arriba + abajo) / 2.
        for m in range(tamanio_de_capa):
            if (n < len(tamanos_de_capa) - 1):
                if (m < ( tamanio_de_capa - 2 )):
                    circulo = plt.Circle((n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical),
                                            espaciado_vertical / 4., color=color_de_capa, ec='k', zorder=4)
                    ax.add_artist(circulo)
                elif (m == (tamanio_de_capa-1)):
                    circulo = plt.Circle((n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical),
                                            espaciado_vertical / 4., color=color_de_capa, ec='k', zorder=4)
                    ax.add_artist(circulo)
                else:
                    ax.text(n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical,
                            '...', fontsize=20, ha='center', va='center')
            else:
                circulo = plt.Circle((n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical),
                                            espaciado_vertical / 4., color=color_de_capa, ec='k', zorder=4)
                ax.add_artist(circulo)
        m+=1
        wrapped_text = textwrap.wrap(label_de_capa[n], width=10)
        for i, line in enumerate(wrapped_text):
            ax.text(n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical - i * 0.05, line, fontsize=20, ha='center', va='center')

        # ax.text(n * espaciado_horizontal + izquierda, tope_de_capa - m * espaciado_vertical, 'Nombre de capa de prueba', fontsize=20, ha='center', va='center')
    # Conexiones
    for n, (tamanio_capa_a, tamanio_capa_b) in enumerate(zip(tamanos_de_capa[:-1], tamanos_de_capa[1:])):
        tope_capa_a = espaciado_vertical * (tamanio_capa_a - 1) / 2. + (arriba + abajo) / 2.
        tope_capa_b = espaciado_vertical * (tamanio_capa_b - 1) / 2. + (arriba + abajo) / 2.
        for m in range(tamanio_capa_a):
            if (m != (tamanio_capa_a-2) ):
                for o in range(tamanio_capa_b):
                    if (o != (tamanio_capa_b-2) ):
                        linea = plt.Line2D([n * espaciado_horizontal + izquierda, (n + 1) * espaciado_horizontal + izquierda],[tope_capa_a - m * espaciado_vertical, tope_capa_b - o * espaciado_vertical], c='k')
                        ax.add_artist(linea)
                    elif (n == (len(tamanos_de_capa) - 2)):
                        linea = plt.Line2D([n * espaciado_horizontal + izquierda, (n + 1) * espaciado_horizontal + izquierda],[tope_capa_a - m * espaciado_vertical, tope_capa_b - o * espaciado_vertical], c='k')
                        ax.add_artist(linea)



fig = plt.figure(figsize=(12, 8))
ax = fig.gca()
ax.axis('off')
dibujar_red_neuronal(ax, .1, .9, .1, .9, [4, 5, 3], ['blue', 'green','red', 'red'],["Input     24","Dense     N","Output    3"])  # Configura estos valores como necesites
# dibujar_red_neuronal(ax, .1, .9, .1, .9, [4, 5,5,5,5, 3], ['blue', 'green','green', 'green','green', 'red'],["Input     24","Dense     N","Dropout","Flatten","Dense     N/2","Output    3"])  # Configura estos valores como necesites
# dibujar_red_neuronal(ax, .1, .9, .1, .9, [4, 5,5,5,5,5, 3], ['blue', 'green','green', 'green','green', 'green', 'red'],["Input     24", "Conv1D     N", "Conv1D     N","Dropout","Flatten","Dense     N/2","Output    3"])  # Configura estos valores como necesites
plt.show()
