#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class LaneFollowingNode:
    def __init__(self):
        # Instanciamos CvBridge
        self.bridge = CvBridge()
        # Nos suscribimos al topic del centro del carril
        self.center_sub = rospy.Subscriber("/center", Float32, self.callback)
        # Publicamos en el topic cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.twist = Twist()
        self.error = Float32()

    def callback(self, msg):

        center = msg.data

        # Calcular el error en la posición del centro del carril con respecto al centro de la imagen
        error = center - 319

        self.error.data = error

        # Asignar la velocidad lineal y angular en función del error
        self.twist.linear.x = 0.1

        if error <= -3 or error >= 3:
            self.twist.linear.x = 0.09
            self.twist.angular.z = -float(error) / 100

        # Publicar el comando de velocidad
        self.cmd_vel_pub.publish(self.twist)

        #publicamos el error para graficar
        self.error_pub.publish(self.error)

def main():
    # Inicializamos el nodo
    rospy.init_node('lane_following_node')
    # Creamos una instancia de LaneFollowingNode
    node = LaneFollowingNode()
    # Mantenemos el nodo funcionando hasta que se detenga
    rospy.spin()
    # Cerramos cualquier ventana abierta
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
