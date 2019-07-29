from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer
import channels.layers
import simplejson as json
import time
import threading
import math
import urllib.request
import numpy as np
import os

#Channel REDIS layer
channel_layer = channels.layers.get_channel_layer()

#ROS imports
from master_msgs.msg import traction_Orders, imu_Speed, imu_Magnetism, pots, current, rpm, arm_Orders, connection
from master_msgs.srv import service_enable
import rospy

## ROS Callbacks

def traction_Orders_Callback(param):
	global ultimo_derecho,ultimo_izquierdo,sensibilidad_jk
	ultimo_derecho=param.rpm_r
	ultimo_izquierdo=param.rpm_l
	sensibilidad_jk=param.sensibility
	pass

def IMU_Speed_Callback(param):
	pass

def IMU_Magnetism_Callback(param):
	pass

def pots_Callback(param):
	pass

def	current_Callback(param):
	global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
	L0_current=param.L0_C
	L1_current=param.L1_C
	L2_current=param.L2_C
	R0_current=param.R0_C
	R1_current=param.R1_C
	R2_current=param.R2_C
	pass

def RPM_Callback(param):
	global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
	L0_speed=param.L0_V
	L1_speed=param.L1_V
	L2_speed=param.L2_V
	R0_speed=param.R0_V
	R1_speed=param.R1_V
	R2_speed=param.R2_V
	pass

def arm_Orders_Callback(param):
	pass

def odom_Callback(param):
	global latitude, longitude
	latitude=param.pose.pose.position.x
	longitude=param.pose.pose.position.y

	pass

#ROS Node declarations
rospy.init_node('Django_node', anonymous=True)

#ROS Publishers
pub_Traction_Orders = rospy.Publisher('topic_traction_orders', traction_Orders, queue_size=10)
pub_Arm_Orders = rospy.Publisher('topic_arm_orders', arm_Orders, queue_size=10)
pub_Connection = rospy.Publisher('topic_connection', connection, queue_size=10)

#ROS Subscribers
rospy.Subscriber('topic_traction_orders', traction_Orders, traction_Orders_Callback)
rospy.Subscriber('topic_arm_orders', arm_Orders, arm_Orders_Callback)
rospy.Subscriber('topic_imu_speed', imu_Speed, IMU_Speed_Callback)
rospy.Subscriber('topic_imu_magnetism', imu_Magnetism, IMU_Magnetism_Callback)
rospy.Subscriber('topic_pots', pots, pots_Callback)
rospy.Subscriber('topic_current', current, current_Callback)
rospy.Subscriber('topic_rpm', rpm, RPM_Callback)
rospy.Publisher ('odom', Odometry, odom_Callback)


#### CONSTANTES ####

ACCION_JK = 0
TOGGLE_MOT_STATUS = 0

GUI_UPDATE_RATE = 100E-3

#### VARIABLES ####

## Joystick virtual
global ultimo_izquierdo, ultimo_derecho, sensibilidad_jk

ultimo_izquierdo = 999
ultimo_derecho = 999
sensibilidad_jk = 0

## Odometria

global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
global L0_status, L1_status, L2_status, R0_status, R1_status, R2_status
global rover_temp, bat0, bat1, bat2, bat3

L0_speed = L1_speed = L2_speed = R0_speed = R1_speed = R2_speed = 0
L0_current = L1_current = L2_current = R0_current = R1_current = R2_current = 0
L0_status = L1_status = L2_status = R0_status = R1_status = R2_status = True
rover_temp = 0
bat0 = bat1 = bat2 = bat3 = 0

## GPS e IMU

global latitude, longitude, azimuth, l_speed, steering_spd, latitude_start, longitude_start

latitude = 4.6030268
longitude = -74.0650463
latitude_start = 4.6030268
longitude_start = -74.0650463
azimuth = 0
l_speed = 0
steering_spd = 0

## Control autonomo

global landmarksAutonomous, landmarksReached, autonomousStatus, updateAutonomous, controlAutonomoActivo

AUTO_DISABLED = 0
AUTO_CHECK_STATUS = 1
AUTO_GOING_TO_LANDMARK = 2
AUTO_STANDBY = 3

landmarksAutonomous = [[4.6029687,-74.0653395], [4.6034687,-74.0653395], [4.6030687,-74.0659395], [4.6040687,-74.0659395]]
landmarksReached = []
autonomousStatus = AUTO_DISABLED

DELETE_COORDINATES = 0
ADD_COORDINATES = 1
EDIT_COORDINATES = 2
STOP = 3
START = 4
CONTINUE = 5

updateAutonomous = True
controlAutonomoActivo = False

## Articulaciones del brazo

global joint0, joint1, joint2, joint3, joint4, joint5, joint6

joint0 = 0
joint1 = 0
joint2 = 0
joint3 = 0
joint4 = 0
joint5 = 0
joint6 = 0



SEPARADOR_POSITIVO = "#"
SEPARADOR_NEGATIVO = "!"






### THREAD QUE INDICA CONEXION AL ROVER ####

def enviarMensajeInicializacion():
	timesToSend = 5
	while timesToSend>0:
		print("Publishing ROS node status (alive)") #Publish active status and motors enable
		timesToSend-=1
		time.sleep(1)
threading.Thread(target=enviarMensajeInicializacion).start()

#### INICIO DJANGO ####
#### CONSUMER DE ACTUALIZACION PARA LA INTERFAZ DE TRACCION ####
class bgUpdate_traction(WebsocketConsumer):
	def connect(self):
		self.room_name = 'r'+str(time.time())
		self.room_group_name = 'bgUpdateConsumers_traction'
		async_to_sync(self.channel_layer.group_add)(self.room_group_name, self.channel_name)
		self.accept()

	def disconnect(self, close_code):
		async_to_sync(self.channel_layer.group_discard)(self.room_group_name,self.channel_name)

	def receive(self, text_data):
		text_data_json = json.loads(text_data)
		if text_data_json['type'] == ACCION_JK:
			procesarJoystick(text_data_json['deltaX'], text_data_json['deltaY'], text_data_json['sensibilidad'])

	def updateGUI(self, event):
		self.send(text_data=json.dumps(event))


#### CONSUMER DE ACTUALIZACION PARA LA INTERFAZ DE ESTATUS ####
class bgUpdate_status(WebsocketConsumer):
	def connect(self):
		self.room_name = 'e'+str(time.time())
		self.room_group_name = 'bgUpdateConsumers_status'
		async_to_sync(self.channel_layer.group_add)(self.room_group_name, self.channel_name)
		self.accept()

	def disconnect(self, close_code):
		async_to_sync(self.channel_layer.group_discard)(self.room_group_name,self.channel_name)

	def receive(self, text_data):
		global L0_status, L1_status, L2_status, R0_status, R1_status, R2_status

		text_data_json = json.loads(text_data)

		print(text_data_json)

		if text_data_json['type'] == TOGGLE_MOT_STATUS:

			mot_id = text_data_json['id']
			new_state = text_data_json['state']

			L0_status = new_state if mot_id == 0 else L0_status
			L1_status = new_state if mot_id == 1 else L1_status
			L2_status = new_state if mot_id == 2 else L2_status
			R0_status = new_state if mot_id == 3 else R0_status
			R1_status = new_state if mot_id == 4 else R1_status
			R2_status = new_state if mot_id == 5 else R2_status

			charEn = SEPARADOR_POSITIVO if new_state else SEPARADOR_NEGATIVO

			# rospy.wait_for_service('service_enable')
			srv_enable = rospy.ServiceProxy('service_enable', service_enable)
			srv_enable("I"+str(mot_id)+charEn)

			#transmitirMensaje("I"+str(mot_id)+charEn)


	def updateGUI(self, event):
		self.send(text_data=json.dumps(event))


#### CONSUMER DE ACTUALIZACION PARA LA INTERFAZ DE AUTONOMO ####
class bgUpdate_autonomous(WebsocketConsumer):
	def connect(self):
		global updateAutonomous
		updateAutonomous = True

		self.room_name = 'e'+str(time.time())
		self.room_group_name = 'bgUpdateConsumers_autonomous'

		async_to_sync(self.channel_layer.group_add)(self.room_group_name, self.channel_name)

		self.accept()

	def disconnect(self, close_code):
		async_to_sync(self.channel_layer.group_discard)(self.room_group_name,self.channel_name)




	def receive(self, text_data):

		global landmarksAutonomous, landmarksReached, autonomousStatus, updateAutonomous, controlAutonomoActivo

		text_data_json = json.loads(text_data)

		if text_data_json['type'] == DELETE_COORDINATES:
			landmarksAutonomous.remove(landmarksAutonomous[text_data_json['number']])
			updateAutonomous = True
		elif text_data_json['type'] == ADD_COORDINATES:
			landmarksAutonomous.append([text_data_json['latitude'], text_data_json['longitude']])
			updateAutonomous = True
		elif text_data_json['type'] == EDIT_COORDINATES:
			landmarksAutonomous[text_data_json['number']] = [text_data_json['latitude'], text_data_json['longitude']]
			updateAutonomous = True
		elif text_data_json['type'] == STOP:
			autonomousStatus = AUTO_DISABLED
			controlAutonomoActivo = False
			updateAutonomous = True
		elif text_data_json['type'] == START and len(landmarksAutonomous)>0:
			autonomousStatus = AUTO_GOING_TO_LANDMARK
			updateAutonomous = True
			controlAutonomoActivo = True
		elif text_data_json['type'] == CONTINUE:
			autonomousStatus = AUTO_GOING_TO_LANDMARK
			updateAutonomous = True




	def updateGUI(self, event):

		self.send(text_data=json.dumps(event))



#### CONSUMER DE ACTUALIZACION PARA LA INTERFAZ DE BRAZO ROBOTICO ####
class bgUpdate_roboticArm(WebsocketConsumer):
	def connect(self):
		self.room_name = 'e'+str(time.time())
		self.room_group_name = 'bgUpdateConsumers_roboticArm'
		async_to_sync(self.channel_layer.group_add)(self.room_group_name, self.channel_name)
		self.accept()
		print("Que mas mi pez")

	def disconnect(self, close_code):
		print("Cual fue el lio mi perro")
		async_to_sync(self.channel_layer.group_discard)(self.room_group_name,self.channel_name)


	def receive(self, text_data):
		text_data_json = json.loads(text_data)
		letrasMotores = ['B', 'C', 'D', 'E', 'F', 'G', 'H']
		print("Que mas mi perro")

		if text_data_json['type'] != "PINZA":
			id_motor = int(text_data_json['id'])
		
			if (id_motor == 4 or id_motor == 5 or id_motor == 6 or id_motor == 2):
				speed = 10
			elif (id_motor == 1 or id_motor == 3):
				speed = 50
		mensaje = ""
		if text_data_json['type'] == "STOP":
			mensaje = letrasMotores[text_data_json['id']-1] + "0" + SEPARADOR_POSITIVO
			# transmitirMensaje(letrasMotores[text_data_json['id']-1] + "0" + SEPARADOR_POSITIVO )
		elif text_data_json['type'] == "ADELANTE":
			mensaje = letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_POSITIVO
			# transmitirMensaje(letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_POSITIVO )
		elif text_data_json['type'] == "ATRAS":
			mensaje = letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_NEGATIVO
			# transmitirMensaje(letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_NEGATIVO )
		elif text_data_json['type'] == "PINZA":
			mensaje = "S" + str(text_data_json['num']) + SEPARADOR_POSITIVO
			# transmitirMensaje("S" + str(text_data_json['num']) + SEPARADOR_POSITIVO )
		msg = arm_Orders()
		msg.message = mensaje# .encode('utf-8')
		pub_Arm_Orders.publish(msg)




	def updateGUI(self, event):

		self.send(text_data=json.dumps(event))



#### THREAD PARA ACTUALIZAR LA INTERFAZ DE TRACCION ####

def threadGUIupdate():

	global ultimo_izquierdo, ultimo_derecho, sensibilidad_jk, latitude, longitude, azimuth, l_speed, steering_spd

	while True:
		options = {}
		options['type'] = 'updateGUI'

		options['PWM_I'] = ultimo_izquierdo
		options['PWM_D'] = ultimo_derecho
		options['sensibilidad_jk_fisico'] = sensibilidad_jk
		options['com'] = True

		options['latitude'] = latitude
		options['longitude'] = longitude
		options['azimuth'] = azimuth
		options['l_speed'] = l_speed
		options['steering'] = steering_spd

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers_traction', options)

		time.sleep(GUI_UPDATE_RATE)

	
threading.Thread(target=threadGUIupdate).start()




#### THREAD PARA ACTUALIZAR LA INTERFAZ DE ESTATUS ####

def threadGUIupdate_STATUS():

	global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
	global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
	global L0_status, L1_status, L2_status, R0_status, R1_status, R2_status
	global rover_temp, bat0, bat1, bat2, bat3

	while True:
		options = {}
		options['type'] = 'updateGUI'

		options['L0_speed'] = L0_speed
		options['L1_speed'] = L1_speed
		options['L2_speed'] = L2_speed
		options['R0_speed'] = R0_speed
		options['R1_speed'] = R1_speed
		options['R2_speed'] = R2_speed
		options['L0_current'] = L0_current
		options['L1_current'] = L1_current
		options['L2_current'] = L2_current
		options['R0_current'] = R0_current
		options['R1_current'] = R1_current
		options['R2_current'] = R2_current
		options['L0_status'] = L0_status
		options['L1_status'] = L1_status
		options['L2_status'] = L2_status
		options['R0_status'] = R0_status
		options['R1_status'] = R1_status
		options['R2_status'] = R2_status
		options['rover_temp'] = rover_temp
		options['bat0'] = bat0
		options['bat1'] = bat1
		options['bat2'] = bat2
		options['bat3'] = bat3

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers_status', options)

		time.sleep(GUI_UPDATE_RATE)

	
threading.Thread(target=threadGUIupdate_STATUS).start()



#### THREAD PARA ACTUALIZAR LA INTERFAZ DE MODO AUTONOMO ####
def threadGUIupdate_AUTONOMOUS():

	global landmarksAutonomous, landmarksReached, autonomousStatus, updateAutonomous, latitude, longitude

	while True:
		options = {}
		options['type'] = 'updateGUI'

		options['landmarksAutonomous'] = landmarksAutonomous
		options['landmarksReached'] = landmarksReached
		options['autonomousStatus'] = autonomousStatus
		options['updateAutonomous'] = updateAutonomous
		options['latitude'] = latitude
		options['longitude'] = longitude
		options['latitude_start'] = latitude_start
		options['longitude_start'] = longitude_start

		updateAutonomous = False
		

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers_autonomous', options)

		time.sleep(GUI_UPDATE_RATE)

	
threading.Thread(target=threadGUIupdate_AUTONOMOUS).start()



#### THREAD PARA ACTUALIZAR LA INTERFAZ DE BRAZO ####

def threadGUIupdate_roboticArm():

	global joint0, joint1, joint2, joint3, joint4, joint5, joint6

	while True:
		options = {}
		options['type'] = 'updateGUI'

		options['joint0'] = joint0
		options['joint1'] = joint1
		options['joint2'] = joint2
		options['joint3'] = joint3
		options['joint4'] = joint4
		options['joint5'] = joint5
		options['joint6'] = joint6

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers_roboticArm', options)

		time.sleep(GUI_UPDATE_RATE)

	
threading.Thread(target=threadGUIupdate_roboticArm).start()


#### METODO PARA RECIBIR LAS ACCIONES DE UN JOYSTICK ####
#Requiere convertir x, y entre -1 y 1
#Requiere filtrar las acciones para no hacerlas muy seguido
def procesarJoystick(x, y, sensibilidad):

	global ultimo_izquierdo, ultimo_derecho

	(calc_PWM_izq, calc_PWM_der) = steering(x, y, sensibilidad)
	
	StringIzquierda = ("L"+str(calc_PWM_izq)+SEPARADOR_NEGATIVO) if (calc_PWM_izq >= 0) else ("L"+str(-calc_PWM_izq)+SEPARADOR_POSITIVO)
	StringDerecha = ("R"+str(calc_PWM_der)+SEPARADOR_NEGATIVO) if (calc_PWM_der >= 0) else ("R"+str(-calc_PWM_der)+SEPARADOR_POSITIVO)

	MensajeSeguridadMotores = ""

	if np.sign(ultimo_izquierdo) != np.sign(calc_PWM_izq) and calc_PWM_izq!=0 and np.sign(ultimo_izquierdo)!=0:
		MensajeSeguridadMotores+="L0#"

	if np.sign(ultimo_derecho) != np.sign(calc_PWM_der) and calc_PWM_der!=0 and np.sign(ultimo_derecho)!=0:
		MensajeSeguridadMotores+="R0#"

	if np.abs(ultimo_izquierdo-calc_PWM_izq)>3 or np.abs(ultimo_derecho-calc_PWM_der)>3 or (calc_PWM_der==0 and ultimo_derecho!=0) or (calc_PWM_izq==0 and ultimo_izquierdo!=0):
		transmitirMensaje(MensajeSeguridadMotores+StringIzquierda+StringDerecha)

		ultimo_izquierdo = calc_PWM_izq
		ultimo_derecho = calc_PWM_der


#### METODO PARA DETERMINAR LOS PWM A PARTIR DE LAS COORDENADAS DE UN JOYSTICK (Metodo diamante encontrado en internet) ####
def steering(x, y, sensibilidad_rcv):
	# convert to polar
	r = math.hypot(-x, -y)
	t = math.atan2(-y, -x)
	# rotate by 45 degrees
	t += math.pi / 4
	# back to cartesian
	left = r * math.cos(t)
	right = r * math.sin(t)
	# rescale the new coords
	left = left * math.sqrt(2)
	right = right * math.sqrt(2)
	# clamp to -1/+1
	left = max(-1, min(left, 1))
	right = max(-1, min(right, 1))
	return int(sensibilidad_rcv*left), int(sensibilidad_rcv*right)



#ROS auxiliary exit check

def ROS_exit_helper():
	while True:
		if rospy.is_shutdown():
			print("Killing Django...")
			os._exit(0)
		time.sleep(500E-3)
threading.Thread(target=ROS_exit_helper).start()
