# Seguimiento de linea Turtlebot
## Requisitos previos
- Ubuntu 20.04 LTS

## Instalación
1. Instalar ROS Noetic

$ sudo apt update

$ sudo apt upgrade

$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh

$ chmod 755 ./install_ros_noetic.sh 

$ bash ./install_ros_noetic.sh


2. Instalar paquetes ROS Dependientes

$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers


3. Instalar paquetes del Turtlebot3

$ sudo apt remove ros-noetic-dynamixel-sdk

$ sudo apt remove ros-noetic-turtlebot3-msgs

$ sudo apt remove ros-noetic-turtlebot3

$ mkdir -p ~/catkin_ws/src

$ cd ~/catkin_ws/src/

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

$ cd ~/catkin_ws && catkin_make

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


4. Guardar el parametro del modelo Turtlebot3

$ echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

$ source ~/.bashrc


5. Modificar la posición de la camara

Ir al script "turtlebot3_burger_for_autorace.urdf.xacro"

catkin_ws > src > turtlebot3 > turtlebot3_description > urdf > turtlebot3_burger_for_autorace.urdf.xacro

En este script se modifica la linea:

linea 166: origin xyz="0.040 -0.011 0.0130" rpy="0 0.174 0"

En su lugar se introducen estos datos: xyz= “0 -0.011 0.25” rpy= “0 0.70 0”


6. Entra en tu "ROS Workspace"

$cd catkin_ws/src

$catkin_create_pkg my_package rospy cv_bridge sensor_msgs geometry_msgs nav_msgs visualization_msgs


7. Clonar el repositorio

$git clone https://github.com/harrysantander/Seguimiento_de_linea_turtlebot


8. Compilar
   
$cd && cd catkin_ws && catkin_make

