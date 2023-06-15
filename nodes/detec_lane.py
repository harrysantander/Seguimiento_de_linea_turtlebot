#! /usr/bin/env python3

# Importamos las librerías necesarias
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


# Definimos la clase para el nodo de detección de carriles
class LaneDetectionNode:
    def __init__(self):
        # Instanciamos CvBridge
        self.bridge = CvBridge()
        # Nos suscribimos al topic de la imagen de la cámara
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.center_pub = rospy.Publisher("/center", Float32, queue_size=1)
        self.float = Float32()

    # Definimos la función de callback para el topic de la imagen de la cámara
    def image_callback(self, data):
        try:

            # Convertimos el mensaje de imagen ROS a imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            ##cv2.imshow("Original", frame)

            # Redimensionamos la imagen
            img = cv2.resize(frame, (640,480))
            ##cv2.imshow("Imagen redimensionada", img)

            # Procesamos la imagen para la detección de carriles
            processed_img = self.process_image(img)

            # Mostramos la imagen resultante
            ##cv2.imshow("canny", processed_img)

            # Detectar los carriles
            lane, centre = self.detect_edge(processed_img)

            self.float.data = centre

            #Mezclar las dos imagenes, la original y la que contiene la linea central
            result = cv2.addWeighted(img, 0.8, lane, 1, 0)
            

            cv2.imshow("Bordes repintados", result)
            cv2.waitKey(1)

            if not rospy.is_shutdown():
                self.center_pub.publish(self.float)

        except CvBridgeError as e:
            # Imprimimos la excepción
            print(e)

    # Función para procesar la imagen
    def process_image(self, img):

        # Convertimos la imagen a escala de grises
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ##cv2.imshow("Escala de grises", gray)
        
        # Aplicamos desenfoque Gaussiano
        blur = cv2.GaussianBlur(gray, (5, 5), 5)
        ##cv2.imshow("Desenfoque Gaussiano", blur)
        
        # Aplicamos umbral a la imagen
        ret, thresh = cv2.threshold(blur, 200, 300, cv2.THRESH_BINARY)
        ##cv2.imshow("Umbral", thresh)
        # Realizamos la detección de bordes
        canny = cv2.Canny(thresh, 50, 115)
        ##cv2.waitKey(1)

        return canny

    # Función detectar el borde del carril izquierdo y carril derecho
    def detect_edge(self, img):

        img_array = np.array(img)
        middle = img_array.shape[1] // 2
        img_lines = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)

        prev_edge_left = None
        prev_edge_right = None
        center = None

        # Recorrer del centro de una fila hacia la izquierda hasta encontrar 
        # el borde del carril izquierdo e ir aumentando la fila y asi guardar 
        # puntos del carril izquierdo
        for y in range(299, img_array.shape[0], +60):
            edge_left = None
            edge_right = None

            # Del centro hacia la izquierda
            for x in range(middle, -1, -1):
                if img_array[y, x] != 0:  # Borde izquierdo encontrado
                    edge_left = (x, y)
                    #print("borde izquierdo (x,y)=", edge_left)
                    break
        
            # Del centro hacia la derecha
            for x in range(middle, img_array.shape[1]):
                if img_array[y, x] != 0:  # Borde derecho encontrado
                    edge_right = (x, y)
                    #print("borde derecho (x,y)=", edge_right)
                    break
            
            # Dibujar los bordes del caril izquierdo y derecho
            if prev_edge_left is not None and edge_left is not None:
                # Dibujar línea verde entre bordes izquierdos
                cv2.line(img_lines, prev_edge_left, edge_left, (0, 255, 0), 20)  

            if prev_edge_right is not None and edge_right is not None:
                # Dibujar línea roja entre bordes derechos
                cv2.line(img_lines, prev_edge_right, edge_right, (0, 255, 0), 20)  

            prev_edge_left = edge_left
            prev_edge_right = edge_right

            # Calculamos el centro del carril
            center = self.calc_center(y, edge_left, edge_right, center, img)

            cv2.circle(img_lines,(center,480),8,(0,0,255),-1)
            cv2.circle(img_lines,(319,480),8,(45,230,255),-1)

            

        return img_lines, center
    
    # Función Calcula el centro del carril capartir de los bordes de la linea mas cercana al robot en este caso es la linea 479 de la imagen
    def calc_center(self, y, edge_left, edge_right, center, canny):
        
        histogram = np.sum(canny[canny.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint   
  
        
        if y == 479:
                
            if edge_left is not None and edge_right is not None:
                center = ((edge_left[0] + edge_right[0]) // 2)
                #print ("centro x=", center)
                
            elif edge_left is None:
                if left_base >= 200:
                    left_base = 0
                center = ((left_base + edge_right[0]) // 2)

            elif edge_right is None:
                if right_base <= 400:
                    right_base = 639
                center = ((edge_left[0] + right_base) // 2)

        return center


def main():
    # Inicializamos el nodo
    rospy.init_node('lane_detection_node')
    # Creamos una instancia de LaneDetectionNode
    node = LaneDetectionNode()
    # Mantenemos el nodo funcionando hasta que se detenga
    rospy.spin()
    # Cerramos cualquier ventana abierta
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
