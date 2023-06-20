# Seguimiento_de_linea_turtlebot
A.	Detección de línea

Primero se inicializa el nodo “lane_detection_node”, este nodo se suscribe al tópico /camera/image, el cual transporta el mensaje de la cámara. El mensaje se lleva a la función image_callback.

1.	Redimensionar y procesar la imagen

En la siguiente imagen (Figura 4) se compara la imagen original que muestra la cámara (izquierda) con la imagen redimensionada (derecha). Esto se realiza para poder ver mucho mejor los cambios realizados y para más comodidad. Las dimensiones de la imagen izquierda son x=320, y=240, y se redimensiona a x=640, y=480
 
Figura 4. Imagen original y redimensionada
Después de redimensionar la imagen, se convierte la imagen a escala de grises (Figura 5).
Convertir la imagen a escala de grises significa que cada píxel de la imagen se reduce a un solo valor de intensidad, en lugar de los tres valores de intensidad (rojo, verde y azul) que se utilizan en una imagen a color. Esto simplifica la imagen y la hace más fácil de procesar en algunos casos.
 
Figura 5. Escala de grises
Seguidamente se aplica un desenfoque gaussiano que suaviza la imagen en escala de grises (Figura 6).
El desenfoque gaussiano toma cada píxel y lo reemplaza por un valor calculado a partir de los píxeles circundantes, ponderados de acuerdo a una distribución gaussiana. Esto tiene el efecto de suavizar la imagen y reducir el ruido y los detalles finos. Los píxeles en áreas de intensidad uniforme (donde los píxeles vecinos tienen valores de intensidad similares) permanecerán casi iguales, mientras que los píxeles en los bordes (donde hay un cambio de intensidad) se suavizarán.
 
Figura 6. Desenfoque gaussiano
Ahora se convierte la imagen en una imagen binaria (Figura 7) aplicando un umbral (threshold). Se elige un valor de umbral, y todos los píxeles de la imagen que tienen un valor de intensidad por encima de ese umbral se convierten a un color (generalmente blanco), y todos los píxeles que tienen un valor de intensidad por debajo de ese umbral se convierten a otro color (generalmente negro). 
El resultado de este proceso es una imagen con solo dos colores, lo que la hace muy simple de analizar. Los objetos de la imagen que antes tenían una intensidad de color superior al umbral estarán ahora representados por un color, y el fondo de la imagen estará representado por otro color. 
En este caso los pixeles de los carriles tendrán un valor de 255 y el resto de píxeles 0.
 
Figura 7. Umbral
Puede observar que la imagen es más clara, eliminando las líneas que no son relevantes y centrándose solo en los carriles.
Se procede a aplicar el último procesamiento de la imagen, el cual es canny, que se utiliza para detectar bordes en la imagen (Figura 8).
Canny utiliza un filtro basado en la derivada del Gaussiano para calcular la intensidad del gradiente de cada píxel. Luego realiza una supresión de los máximos no locales para eliminar píxeles que no forman parte de un borde. Por último, utiliza un umbral de histéresis para determinar los bordes finales, lo que resulta en bordes finos y bien definidos.
El resultado de este proceso será una imagen en la que los bordes (las regiones de la imagen donde la intensidad cambia rápidamente) están resaltados.
 
Figura 8. Canny
2.	Obtener las coordenadas de los bordes y calcular el centro del carril
En la función “detect_edge” se crea un bucle for para un rango de 299 hasta la última fila de la imagen. En el lenguaje de programación las matrices de imágenes comienzan desde 0. Por lo tanto, en esta imagen con una dimensión y=480, los índices válidos realmente van desde 0 hasta 479. Es decir, el bucle recorre desde 299 hasta 479. Además, lo hace aumentando la fila de 60 en 60.
A cada una de las filas se le aplica otro bucle for que recorre desde el centro de la fila hacia la izquierda hasta encontrar un valor diferente a 0, y lo mismo para el lado derecho.
Obteniendo así las coordenadas de los bordes (Figura 9).
 
Figura 9. Datos del primer instante de los bordes
En la imagen anterior se puede ver que para la línea 299, se encuentra un borde en el lado izquierdo en x=86 y otro en el lado derecho en x =478. Para la línea 359, se encuentra un borde en el lado izquierdo en x=61 y otro en el lado derecho en x =499…
Para calcular el centro del carril se hace uso de esta función, pero solo para la fila 479, que es la más cercana al robot, se encuentra un borde en el lado izquierdo en x=11 y otro en el lado derecho en x=540, sabiendo el valor de x para ambos bordes se hace una media y se obtiene centro x=275.
Con las coordenadas de los bordes se dibuja una línea de color verde para cada borde. Obteniendo así la siguiente imagen (Figura 10).
 
Figura 10. Bordes dibujados en verde
Y el valor del centro x= 275, se publica en el tópico /center.
Las imágenes mostradas son solo una captura del primer instante donde se inicia el robot.
