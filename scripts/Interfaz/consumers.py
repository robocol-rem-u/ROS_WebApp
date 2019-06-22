from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer
import channels.layers
import simplejson as json
import time
import threading
import _thread
import socket
import serial
import math
import urllib.request
import pygame
import numpy as np

from std_msgs.msg import String
import rospy

channel_layer = channels.layers.get_channel_layer()

rospy.init_node('Django_node', anonymous=False)
tractionPublisher = rospy.Publisher('tractionCMD', String, queue_size=10)


#### CONSTANTES ####

HOST = '192.168.0.101'
PORT = 7071
DIRECCION_XBEE = "/dev/tty.usbserial-A100RG3E"
BAUD_RATE = 115200
SEPARADOR_POSITIVO = "#"
SEPARADOR_NEGATIVO = "!"
RF = False
WIFI = True
DIR_CAM0_FOSCAM = "http://192.168.0.120:57055"
ACCION_JK = 0
COM_CHANGE = 1
ACTIVAR_JK = True
TOGGLE_MOT_STATUS = 0

#### VARIABLES ####

lock = threading.Lock()

global rf_wifi_selector, EnviarMensaje, ultimo_izquierdo, ultimo_derecho, sensibilidad_jk

rf_wifi_selector = RF
EnviarMensaje = True
ultimo_izquierdo = 999
ultimo_derecho = 999
sensibilidad_jk = 1

global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
global L0_status, L1_status, L2_status, R0_status, R1_status, R2_status
global rover_temp, bat0, bat1, bat2, bat3

L0_speed = L1_speed = L2_speed = R0_speed = R1_speed = R2_speed = 0
L0_current = L1_current = L2_current = R0_current = R1_current = R2_current = 0
L0_status = L1_status = L2_status = R0_status = R1_status = R2_status = True
rover_temp = 0
bat0 = bat1 = bat2 = bat3 = 0

global latitude, longitude, azimuth, l_speed, steering_spd, latitude_start, longitude_start

latitude = 4.6030268
longitude = -74.0650463
latitude_start = 4.6030268
longitude_start = -74.0650463
azimuth = 0
l_speed = 0
steering_spd = 0

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

####### TO TEST ######
# bat0 = 4.2*6
# bat1= 3.6*6
# bat2= 3.4*6
# bat3= 3.2*6
# L0_speed = 1
# L1_speed = 2
# L2_speed = 3
# R0_speed = 4
# R1_speed = 5
# R2_speed = 6
# L0_current = 7
# L1_current = 8
# L2_current = 9
# R0_current = 10
# R1_current = 11
# R2_current = 12
# rover_temp = 13

# L0_status = True
# L1_status = True
# L2_status = True
# R0_status = True
# R1_status = True
# R2_status = True


global joint0, joint1, joint2, joint3, joint4, joint5, joint6

joint0 = 0
joint1 = 0
joint2 = 0
joint3 = 0
joint4 = 0
joint5 = 0
joint6 = 0

#### INICIALIZACION DE CONEXIONES ####

CONECTAR_XBEE = False
#CONECTAR_WIFI = True
CONECTAR_WIFI = False

if CONECTAR_XBEE:
	try:
		xBeeSerial = serial.Serial(DIRECCION_XBEE, baudrate=BAUD_RATE)
		print("\t \t -- Conexion con XBee S8 inicializada --")
	except:
		print("\033[1;31mNo se pudo inicializar la conexión con la XBee S8 \033[0;0m")
		_thread.interrupt_main()
if CONECTAR_WIFI:
	s = socket.socket()
	s.settimeout(1)
	try:
		s.connect((HOST, PORT))
		s.send(str.encode("C+RF"))
	except:
		print("\033[1;31mError en conexión Wi-Fi \033[0;0m")
		_thread.interrupt_main()
	s.close()
if not (CONECTAR_WIFI and CONECTAR_XBEE):
	print("\033[1;31mNo estan activas todas las conexiones, encontrara funcionalidad limitada \033[0;0m")
print("\t \t -- Modo de control RF --")


#### METODO PARA TRANSMITIR MENSAJES WIFI - RF ####

def transmitirMensaje(mensaje):
	retornar = True
	lock.acquire()
	global rf_wifi_selector
	if rf_wifi_selector == WIFI:
		s = socket.socket()
		s.settimeout(1)
		try:
			s.connect((HOST, PORT))
			s.send((mensaje).encode())
		except:
			print("\033[1;31mError en conexión Wi-Fi \033[0;0m")
			retornar = False
		s.close()
	elif rf_wifi_selector == RF:
		try:
			xBeeSerial.write((mensaje).encode())
		except:
			print("\033[1;31mError en conexión con Xbee S8 \033[0;0m")
			retornar = False
	else:
		print("\033[1;31mError \033[0;0m")
	print("Comando transmitido:",mensaje,"Estado:",retornar)
	lock.release()
	return retornar


### THREAD QUE INDICA CONEXION AL ROVER ####

def enviarMensajeInicializacion():
	global EnviarMensaje
	while EnviarMensaje:
		nMsg = 1#3
		print("Enviando Inicialización")
		transmitirMensaje("A"+str(nMsg)+"#I0#I1#I2#I3#I4#I5#")
		time.sleep(1)
#threading.Thread(target=enviarMensajeInicializacion).start()

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
		elif text_data_json['type'] == COM_CHANGE:
			cambiarEstado(text_data_json['newCom'])

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

			transmitirMensaje("I"+str(mot_id)+charEn)


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

	def disconnect(self, close_code):
		async_to_sync(self.channel_layer.group_discard)(self.room_group_name,self.channel_name)

	def receive(self, text_data):

		text_data_json = json.loads(text_data)

		letrasMotores = ['B', 'C', 'D', 'E', 'F', 'G', 'H']

		if text_data_json['type'] != "PINZA":
			id_motor = int(text_data_json['id'])
		
			if (id_motor == 4 or id_motor == 5 or id_motor == 6 or id_motor == 2):
				speed = 10
			elif (id_motor == 1 or id_motor == 3):
				speed = 50

		if text_data_json['type'] == "STOP":
			transmitirMensaje(letrasMotores[text_data_json['id']-1] + "0" + SEPARADOR_POSITIVO )
		elif text_data_json['type'] == "ADELANTE":
			transmitirMensaje(letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_POSITIVO )
		elif text_data_json['type'] == "ATRAS":
			transmitirMensaje(letrasMotores[text_data_json['id']-1] + str(speed) + SEPARADOR_NEGATIVO )
		elif text_data_json['type'] == "PINZA":
			transmitirMensaje("S" + str(text_data_json['num']) + SEPARADOR_POSITIVO )

	def updateGUI(self, event):

		self.send(text_data=json.dumps(event))



#### THREAD PARA ACTUALIZAR LA INTERFAZ DE TRACCION ####

def threadGUIupdate():

	global ultimo_izquierdo, ultimo_derecho, sensibilidad_jk, rf_wifi_selector, latitude, longitude, azimuth, l_speed, steering_spd

	while True:
		options = {}
		options['type'] = 'updateGUI'

		options['PWM_I'] = ultimo_izquierdo
		options['PWM_D'] = ultimo_derecho
		options['sensibilidad_jk_fisico'] = sensibilidad_jk
		options['com'] = rf_wifi_selector

		options['latitude'] = latitude
		options['longitude'] = longitude
		options['azimuth'] = azimuth
		options['l_speed'] = l_speed
		options['steering'] = steering_spd

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers_traction', options)

		time.sleep(100E-3)

	
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

		time.sleep(100E-3)

	
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

		time.sleep(100E-3)

	
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

		time.sleep(100E-3)

	
threading.Thread(target=threadGUIupdate_roboticArm).start()


################### SIMULACION ###################
global inicio
inicio = time.time()
def simu():
	global latitude, longitude, latitude_start, longitude_start, inicio, autonomousStatus,landmarksReached, landmarksAutonomous, updateAutonomous

	while True:

		if autonomousStatus == AUTO_DISABLED:
			latitude = 4.6030268
			longitude = -74.0650463
			latitude_start = latitude
			longitude_start = longitude
			landmarksReached = []
			primerVez = True
		elif autonomousStatus == AUTO_GOING_TO_LANDMARK:


			if (not len(landmarksReached)==len(landmarksAutonomous)-1) or (len(landmarksAutonomous)==1 and primerVez):

				if not primerVez:
					landmarksReached.append(len(landmarksReached))
				

				longitud_autonomous_reached = len(landmarksReached)

				if longitud_autonomous_reached!=0:
					latitude = ( landmarksAutonomous[longitud_autonomous_reached-1][0] + landmarksAutonomous[longitud_autonomous_reached][0] ) / 2
					longitude = ( landmarksAutonomous[longitud_autonomous_reached-1][1] + landmarksAutonomous[longitud_autonomous_reached][1] ) / 2
				else:
					latitude = ( latitude_start + landmarksAutonomous[0][0] ) / 2
					longitude = ( longitude_start + landmarksAutonomous[0][1] ) / 2

				time.sleep(1.5)

			else:
				autonomousStatus = AUTO_DISABLED



			if not autonomousStatus == AUTO_DISABLED:

				if longitud_autonomous_reached!=len(landmarksAutonomous):
					autonomousStatus = AUTO_STANDBY
				else:
					autonomousStatus = AUTO_DISABLED


				latitude = landmarksAutonomous[len(landmarksReached)][0]
				longitude = landmarksAutonomous[len(landmarksReached)][1]


				updateAutonomous = True
				primerVez = False
			

		time.sleep(1E-3)


#threading.Thread(target=simu).start()
################### SIMULACION ###################

#### ENVIAR COMANDOS A CAM0 ####
def enviar_comando_cam0(comando):
	try:
		resultado = urllib.request.urlopen(DIR_CAM0_FOSCAM+"/decoder_control.cgi?command="+comando+"&user=alegis&pwd=alegis",timeout=0.5)
	except:
		print("No se pudo ejecutar el comando")

#### CAMBIAR TIPO DE COMUNICACION ####
def cambiarEstado(nueva_com):
	global rf_wifi_selector
	rf_wifi_selector = nueva_com

	modo = "C+WIFI" if rf_wifi_selector else "C+RF"
	print("Modo de control:",modo)

	lock.acquire()
	s = socket.socket()
	s.settimeout(1)
	try:
		s.connect((HOST, PORT))
		s.send(str.encode(modo))
	except:
		print("\033[1;31mError en conexión Wi-Fi \033[0;0m")
	s.close()
	lock.release()

#### METODO PARA RECIBIR LAS ACCIONES DE UN JOYSTICK ####
#Requiere convertir x, y entre -1 y 1
#Requiere filtrar las acciones para no hacerlas muy seguido
def procesarJoystick(x, y, sensibilidad, sobreEscribirPWM = False, PWM_I = 0, PWM_D = 0):

	global EnviarMensaje, ultimo_izquierdo, ultimo_derecho

	if not sobreEscribirPWM:
		(calc_PWM_izq, calc_PWM_der) = steering(x, y, sensibilidad)
	else:
		(calc_PWM_izq, calc_PWM_der) = (int(PWM_I), int(PWM_D))
	
	StringIzquierda = ("L"+str(calc_PWM_izq)+SEPARADOR_NEGATIVO) if (calc_PWM_izq >= 0) else ("L"+str(-calc_PWM_izq)+SEPARADOR_POSITIVO)
	StringDerecha = ("R"+str(calc_PWM_der)+SEPARADOR_NEGATIVO) if (calc_PWM_der >= 0) else ("R"+str(-calc_PWM_der)+SEPARADOR_POSITIVO)

	MensajeSeguridadMotores = ""

	if np.sign(ultimo_izquierdo) != np.sign(calc_PWM_izq) and calc_PWM_izq!=0 and np.sign(ultimo_izquierdo)!=0:
		MensajeSeguridadMotores+="L0#"

	if np.sign(ultimo_derecho) != np.sign(calc_PWM_der) and calc_PWM_der!=0 and np.sign(ultimo_derecho)!=0:
		MensajeSeguridadMotores+="R0#"

	if np.abs(ultimo_izquierdo-calc_PWM_izq)>3 or np.abs(ultimo_derecho-calc_PWM_der)>3 or (calc_PWM_der==0 and ultimo_derecho!=0) or (calc_PWM_izq==0 and ultimo_izquierdo!=0):
		EnviarMensaje = not transmitirMensaje(MensajeSeguridadMotores+StringIzquierda+StringDerecha)

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



#### THREAD PARA DETECCION DEL JOYSTICK FISICO ####

def controlJoystick():
	global sensibilidad_jk, rf_wifi_selector
	while True:
		for event in pygame.event.get():

			if event.type == pygame.JOYBUTTONDOWN:
				if event.button == 2:
					cambiarEstado(not rf_wifi_selector)
			else:

				sensibilidad_jk = int(250 - 125*(1+my_joystick.get_axis(3)))

				if np.linalg.norm((my_joystick.get_axis(1), my_joystick.get_axis(0))) < 0.05 and np.abs(my_joystick.get_axis(2))>0.1:
					procesarJoystick(0, 0, sensibilidad_jk, True, sensibilidad_jk*my_joystick.get_axis(2), -sensibilidad_jk*my_joystick.get_axis(2))
				elif np.linalg.norm((my_joystick.get_axis(1), my_joystick.get_axis(0))) > 0.05:
					procesarJoystick(my_joystick.get_axis(1), my_joystick.get_axis(0), sensibilidad_jk)
				else:
					procesarJoystick(0, 0, sensibilidad_jk, True, 0, 0)

			clock.tick(20)



#### INICIAR THREAD PARA DETECCION DEL JOYSTICK FISICO ####
if ACTIVAR_JK:
	try:
		pygame.init()
		print("Joysticks: ", pygame.joystick.get_count())
		my_joystick = pygame.joystick.Joystick(0)
		my_joystick.init()
		clock = pygame.time.Clock()
		threading.Thread(target=controlJoystick).start()
	except:
		print("\033[1;31mJoystick no encontrado \033[0;0m")


### ACTUALIZACION DE TELEMETRIA


#s = socket.socket()
#host = '192.168.0.999'
#port = 7080
#s.bind((host, port))
#s.listen(5)

def ThreadEnviarAInterfaz():
	global latitude, longitude, azimuth, l_speed, steering_spd, latitude_start, longitude_start
	global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
	global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
	global rover_temp, bat0, bat1, bat2, bat3
	global joint0, joint1, joint2, joint3, joint4, joint5, joint6
	while True:
		try:
			c, addr = s.accept()
			rcv = c.recv(4096).decode()
			data_rcv_split = rcv.split('\n')
			data_rcv_split = [float(x) for x in data_rcv_split]

			(latitude, longitude, azimuth, l_speed, steering_spd, L2_speed, L1_speed, L0_speed, R2_speed, R1_speed, R0_speed, L2_current, L0_current, L1_current, R2_current, R1_current, R0_current, rover_temp, bat0, bat1, bat2, bat3, joint0, joint1, joint2, joint3, joint4, joint5, joint6) = data_rcv_split
			
			(L0_speed, L1_speed, L2_speed) = (-L0_speed, -L1_speed, -L2_speed)

			(L0_current, L2_current, L1_current, R0_current, R1_current, R2_current) = (L0_current/140, L2_current/140, L1_current/140, R0_current/140, R1_current/140, R2_current/140)


			if not controlAutonomoActivo:
				latitude_start = latitude
				longitude_start = longitude

		except:
			print("Error de protocolo")

		c.close()

#threading.Thread(target=ThreadEnviarAInterfaz).start()




