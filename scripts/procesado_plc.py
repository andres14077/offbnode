#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import copy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PolygonStamped,Point32
from offbnode.msg import PlaneStamped
# import matplotlib.pyplot as plt
# import statistics
import numpy as np
import csv
import rospkg
import tensorflow as tf
from offbnode.msg import TypePlane

gpu_devices = tf.config.experimental.list_physical_devices('GPU')
for device in gpu_devices:
    tf.config.experimental.set_memory_growth(device, True)


class procesado_plc:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.planos=[]
        self.fondos=[]
        # procesado de nube de puntos para plano
        self.cmd_sub=rospy.Subscriber("offbnode/procesado_on", Bool, self.cmd_cb)
        self.plane_sub=rospy.Subscriber('offbnode/plane_individual_in_map', PlaneStamped, self.plane_individual_in_map_cb)

        self.plano_in_local_pub=rospy.Publisher('offbnode/plano_in_local', PlaneStamped, queue_size=10)
        self.procesado_completed_pub=rospy.Publisher('offbnode/procesado_completed', Bool, queue_size=10 )
        self.plane_pub=rospy.Publisher('offbnode/plano_promedio_in_map', PlaneStamped, queue_size=10)
        # procesado de nube de puntos para imagen de profundidad
        self.point_cloud_depth_sub=rospy.Subscriber("/offbnode/depth/points", PointCloud2, self.depth_cloud_cb,queue_size=1)
        self.point_cloud_depth_start_sub=rospy.Subscriber("/offbnode/depth_start", Empty, self.depth_cloud_start_cb)
        self.point_cloud_depth_finish_sub=rospy.Subscriber("/offbnode/depth_finish", Empty, self.depth_cloud_finish_cb)

        self.point_cloud_depth_ok_pub=rospy.Publisher('offbnode/depth_ok', Empty, queue_size=10)
        self.resultado_tipo_plano_pub=rospy.Publisher('offbnode/tipo_plano', TypePlane, queue_size=10)
        self.poligono_fondo_pub = rospy.Publisher("offbnode/poligono_fondo", PolygonStamped,queue_size=10)

        rospack = rospkg.RosPack()
        # self.archivo = rospack.get_path('offbnode')+'/database.csv'
        # rospy.loginfo(self.archivo)

        self.clasificador = tf.keras.models.load_model(rospack.get_path('offbnode')+'/neural_model/ANN2_64.h5')
        self.class_labels=['ladera','pradera','valle']

    def cmd_cb(self,msg):
        self.point_cloud_sub=rospy.Subscriber("offbnode/points2", PointCloud2, self.point_cloud_cb,queue_size=1)

    def Plano_Z(self,p,v,x,y):
        return p.z-(v.x*(x-p.x)+v.y*(y-p.y))/v.z

    def find_local_minima_with_position(self,n):
        min_separation=6
        # Asegurarnos de que hay al menos 3 puntos para comparar
        if len(self.fondos) < 3:
            return []
        minima_positions = []
        # Comparar cada punto con su vecino izquierdo y derecho
        for i in range(len(self.fondos)):
            left = self.fondos[i - 1] if i != 0 else self.fondos[-1]
            right = self.fondos[i + 1] if i != len(self.fondos) - 1 else self.fondos[0]
            if self.fondos[i] < left and self.fondos[i] < right:
                minima_positions.append(i)
        # Ordenar por valor y tomar los primeros n mínimos
        minima_positions.sort(key=lambda x: self.fondos[x])
        # Filtrar los mínimos para asegurar una separación mínima
        filtered_minima = []
        for pos in sorted(minima_positions, key=lambda x: self.fondos[x]):
            too_close = False
            for other in filtered_minima:
                # Calcular la distancia cíclica teniendo en cuenta la naturaleza circular del vector
                distance = min(abs(pos - other), len(self.fondos) - abs(pos - other))
                if distance < min_separation:
                    too_close = True
                    break
            if not too_close:
                filtered_minima.append(pos)
            if len(filtered_minima) == n:
                break
        return filtered_minima

    def point_cloud_cb(self,msg):
        rospy.loginfo("regresion lineal de nube de puntos")
        self.point_cloud_sub.unregister()
        plane=PlaneStamped()
        z=[]
        x=[]
        y=[]
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            z.append(p[2])
            x.append(p[0])
            y.append(p[1])
        X = np.array([x,y])
        X = np.insert(X, 0, np.array((np.ones(len(X[0])))), 0).T
        Z = np.array(z)
        b = np.linalg.inv(X.T @ X) @ X.T @ Z
        rospy.logdebug("resultado de regresion lineal")
        rospy.logdebug(b)
        plane.header.frame_id="cgo3_camera_optical_link"
        plane.header.stamp = rospy.Time.now()
        plane.point.point.z = b[0]
        plane.vector.vector.x = b[1]*-1
        plane.vector.vector.y = b[2]*-1
        plane.vector.vector.z = 1
        self.plano_in_local_pub.publish(plane)

    def plane_individual_in_map_cb(self,msg):
        rospy.loginfo("regresion lineal de planos adquiridos, plano promedio")
        self.planos.append(copy.deepcopy(msg))
        plane=PlaneStamped()
        z=[]
        x=[]
        y=[]
        for i in self.planos:
            for j in range(-20,20):
                for k in range(-20,20):
                    x.append(j)
                    y.append(k)
                    z.append(self.Plano_Z(i.point.point,i.vector.vector,j,k))
        X = np.array([x,y])
        X = np.insert(X, 0, np.array((np.ones(len(X[0])))), 0).T
        Z = np.array(z)
        b = np.linalg.inv(X.T @ X) @ X.T @ Z
        plane.header.frame_id="map"
        plane.header.stamp = rospy.Time.now()
        plane.point.point.z = b[0]
        plane.vector.vector.x = b[1]*-1
        plane.vector.vector.y = b[2]*-1
        plane.vector.vector.z = 1
        self.plane_pub.publish(plane)
        procesado_completed=Bool(True)
        self.procesado_completed_pub.publish(procesado_completed)

    def depth_cloud_cb(self,msg):
        rospy.logdebug("calculando fondo en nube de puntos")
        z=[]
        for p in pc2.read_points(msg, field_names = ("z"), skip_nans=True):
            z.append(p[0])
        Z = np.array([z])
        puntos_lejos=0
        depth_max = Z.max()
        for i in z:
            if(i>(depth_max*0.8)):
                puntos_lejos+=1;
        puntos_lejos=puntos_lejos*100.0/len(z)
        rospy.logdebug("valor fondo")
        rospy.logdebug(puntos_lejos)
        self.fondos.append(puntos_lejos)
        self.point_cloud_depth_ok_pub.publish()

    def depth_cloud_start_cb(self,msg):
        self.fondos=[]

    def depth_cloud_finish_cb(self,msg):
        # abre el archivo CSV en modo de escritura al final (append)
        # with open(self.archivo, 'a', newline='') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=',')
        #     writer.writerow(self.fondos)  # escribe la nueva línea al final del archivo CSV
        respuesta = TypePlane()

        poligono_fondo = PolygonStamped()

        poligono_fondo.header.stamp=rospy.Time.now()
        poligono_fondo.header.frame_id="map"

        p1 =Point32()
        factor=2*np.pi/len(self.fondos)
        for i in range(len(self.fondos)):
            p1.x=50*np.cos(i*factor)
            p1.y=50*np.sin(i*factor)
            p1.z=100-self.fondos[i]
            poligono_fondo.polygon.points.append(copy.deepcopy(p1))
        self.poligono_fondo_pub.publish(poligono_fondo)
        rospy.logdebug(self.fondos)
        X_tensor=np.reshape(self.fondos,(1, 25))
        X_tensor = tf.convert_to_tensor(X_tensor, dtype=tf.float32)
        y_pred = self.clasificador(X_tensor)
        y_pred_classes = tf.argmax(y_pred, axis=1).numpy()
        rospy.loginfo("Terreno identificado como: "+self.class_labels[y_pred_classes[0]])

        respuesta.header.frame_id="map"
        respuesta.header.stamp = rospy.Time.now()
        respuesta.type=self.class_labels[y_pred_classes[0]]
        if (self.class_labels[y_pred_classes[0]]=='ladera'):
            respuesta.max=self.find_local_minima_with_position(1)
        elif (self.class_labels[y_pred_classes[0]]=='valle'):
            respuesta.max=self.find_local_minima_with_position(2)

        self.resultado_tipo_plano_pub.publish(respuesta)

if __name__ == '__main__':
    rospy.init_node('procesado_plc', anonymous=True)
    nodo=procesado_plc()
    rospy.spin()