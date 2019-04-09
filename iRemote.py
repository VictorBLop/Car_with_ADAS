#Importacion de librerias
import RPi.GPIO as GPIO
import os
import time
import serial
from datetime import datetime
from gpiozero import Buzzer
import cv2
import time
import numpy as np
from cv2 import *


#Definicion de los parametros del PUERTO SERIE
ser=serial.Serial(
        port="/dev/ttyAMA0",
	baudrate=9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
	timeout=0.15
)


#Configuracion de los pines segun el microprocesador Broadcom
GPIO.setmode(GPIO.BCM)


#Definicion de los nombre de los pines de los ultrasonidos
TRIG = 20
ECHO1 = 21
ECHO2 = 13
ECHO3 = 16
ECHO4 = 26

#Configuracion de los pines de los ultrasonidos
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(ECHO3, GPIO.IN)
GPIO.setup(ECHO4, GPIO.IN)


#Definicion de los nombre de los pines del puente H
ena = 18			
in1 = 23
in2 = 24
enb = 19
in3 = 6
in4 = 5
enw = 12

#Configuracion de los pines del puente H
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.setup(enw,GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

#Salidas PWM 
pwm_a = GPIO.PWM(ena,500)
pwm_b = GPIO.PWM(enb,500)

#Inicializacion de los PWM con un duty Cicly de cero
pwm_a.start(0)
pwm_b.start(0)


#Definicion de las funciones de sentido de giro y velocidad de los MOTORES
vel_max = 75
vel_min = 50
#Hacia delante
def  f_fwrd():
	GPIO.output(in1,True)
	GPIO.output(in2,False)
	GPIO.output(in3,True)
	GPIO.output(in4,False)
	vel_a = vel_max
	vel_b = vel_max
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 1
	return vel, vel_a, vel_b, sent

#Hacia atras
def f_back():
	GPIO.output(in1,False)
	GPIO.output(in2,True)
	GPIO.output(in3,False)
	GPIO.output(in4,True)
	vel_a = vel_max
	vel_b = vel_max
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = -1
	return vel, vel_a, vel_b, sent

#Hacia delante-derecha
def  f_fwrdright():
	GPIO.output(in1,True)
	GPIO.output(in2,False)
	GPIO.output(in3,True)
	GPIO.output(in4,False)
	vel_a = vel_max
	vel_b = vel_min
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 1
	return vel, vel_a, vel_b, sent

#Hacia delante-izquierda
def  f_fwrdleft():
	GPIO.output(in1,True)
	GPIO.output(in2,False)
	GPIO.output(in3,True)
	GPIO.output(in4,False)
	vel_a = vel_min
	vel_b = vel_max
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 1
	return vel, vel_a, vel_b, sent

#Hacia atras-derecha
def  f_backright():
	GPIO.output(in1,False)
	GPIO.output(in2,True)
	GPIO.output(in3,False)
	GPIO.output(in4,True)
	vel_a = vel_max
	vel_b = vel_min
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = -1
	return vel, vel_a, vel_b, sent
	
#Hacia detras-izquierda
def  f_backleft():
	GPIO.output(in1,False)
	GPIO.output(in2,True)
	GPIO.output(in3,False)
	GPIO.output(in4,True)
	vel_a = vel_min
	vel_b = vel_max
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = -1
	return vel, vel_a, vel_b, sent
	
#Hacia la izquierda (solo gira rueda derecha)
def  f_left():
	GPIO.output(in1,True)
	GPIO.output(in2,False)
	GPIO.output(in3,False)
	GPIO.output(in4,False)
	vel_a = vel_min
	vel_b = 0
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 1
	return vel, vel_a, vel_b, sent

#Hacia atras-izquierda
def  f_leftpark():
	GPIO.output(in1,False)
	GPIO.output(in2,True)
	GPIO.output(in3,False)
	GPIO.output(in4,False)
	vel_a = vel_min
	vel_b = 0
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = -1
	return vel, vel_a, vel_b, sent

#Hacia la derecha (solo gira rueda izquierda)
def  f_right():
	GPIO.output(in1,False)
	GPIO.output(in2,False)
	GPIO.output(in3,True)
	GPIO.output(in4,False)
	vel_a = 0
	vel_b = vel_min
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 1
	return vel, vel_a, vel_b, sent

#Hacia atras-derecha
def  f_rightpark():
	GPIO.output(in1,False)
	GPIO.output(in2,False)
	GPIO.output(in3,False)
	GPIO.output(in4,True)
	vel_a = 0
	vel_b = vel_min
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = -1
	return vel, vel_a, vel_b, sent

#Ambas ruedas paradas
def  f_stop():
	GPIO.output(in1,False)
	GPIO.output(in2,False)
	GPIO.output(in3,False)
	GPIO.output(in4,False)
	vel_a = 0
	vel_b = 0
	pwm_a.ChangeDutyCycle(vel_a)
	pwm_b.ChangeDutyCycle(vel_b)
	vel = (vel_a + vel_b) / 2
	sent = 0
	return vel, vel_a, vel_b, sent


#Definicion de la funcion del modo PARKING
def parking(lado):
	#Parking a la izquierda
	if lado == "i":
		f_stop()
		time.sleep(1)
		f_leftpark()
		time.sleep(0.5)
		f_rightpark()
		time.sleep(0.5)
		f_stop()

	#Parking a la derecha
	else:
		f_stop()
		time.sleep(1)
		f_rightpark()
		time.sleep(0.5)
		f_leftpark()
		time.sleep(0.5)
		f_stop()


#Definicion del tiempo maximo de ultrasonido
tmax = 0.0017


#inicializacion de variables
act1 = 0
act2 = 0
act3 = 0
act4 = 0
fin1 = 0
fin2 = 0
fin3 = 0
fin4 = 0
vel = 0
vel_a = 0
vel_b = 0
tiempo_inicio1 = 0
tiempo_inicio2 = 0
pulso_inicio1 = 0
pulso_inicio2 = 0
pulso_inicio3 = 0
pulso_inicio4 = 0
sent = 0
aviso_f = 0
aviso_t = 0
aviso_d = 0
aviso_i = 0
claxon_inicio = 0
stop_inicio = 0
luces_inicio = 0
tiempo = 0
signal = 0
cruce = 0

#Variables y parametros de la webcam
area=800000
cont_tr_naranja=0
cont_cd_naranja=0
cont_tr_verde=0
cont_cd_verde=0

#Variables de detección de color.
detec_tr_naranja=0
detec_cd_naranja=0
detec_tr_verde=0
detec_cd_verde=0

captura = VideoCapture(0)
captura.set(cv2.cv.CV_CAP_PROP_FPS, 3)
captura.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 180)
captura.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 144)

#Establecemos los límites del color verde a detectar
verde_bajos=np.array([49,50,50], dtype=np.uint8)
verde_altos=np.array([80,255,255], dtype=np.uint8)
		
#Establecemos los límites del color naranja a detectar
naranja_bajos = np.array([18,40,90], dtype=np.uint8)
naranja_altos = np.array([27,255,255], dtype=np.uint8)

#PROGRAMA PRINCIPAL
try:
	while True:		
		#ULTRASONIDOS
		#Disparo del trigger
		GPIO.output(TRIG, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(TRIG, GPIO.LOW)

		#Cuenta del tiempo
		while True:
			#Ultrasonido 1
			if GPIO.input(ECHO1) == GPIO.LOW and act1 == 0:
				pulso_inicio1 = time.time()
			elif fin1 == 0:
				act1 = 1
				pulso_fin1 = time.time()
				if GPIO.input(ECHO1) == GPIO.LOW or (pulso_fin1 - pulso_inicio1) >= tmax:
					fin1 = 1

			#Ultrasonido 2
			if GPIO.input(ECHO2) == GPIO.LOW and act2 == 0:
				pulso_inicio2 = time.time()
			elif fin2 == 0:
				act2 = 1
				pulso_fin2 = time.time()
				if GPIO.input(ECHO2) == GPIO.LOW or (pulso_fin2 - pulso_inicio2) >= tmax:
					fin2 = 1

			#Ultrasonido 3
			if GPIO.input(ECHO3) == GPIO.LOW and act3 == 0:
				pulso_inicio3 = time.time()
			elif fin3 == 0:
				act3 = 1
				pulso_fin3 = time.time()
				if GPIO.input(ECHO3) == GPIO.LOW or (pulso_fin3 - pulso_inicio3) >= tmax:
					fin3 = 1

			#Ultrasonido 4
			if GPIO.input(ECHO4) == GPIO.LOW and act4 == 0:
				pulso_inicio4 = time.time()
			elif fin4 == 0:
				act4 = 1
				pulso_fin4 = time.time()
				if GPIO.input(ECHO4) == GPIO.LOW or (pulso_fin4 - pulso_inicio4) >= tmax:
					fin4 = 1

			if fin1 == 1 and fin2 == 1 and fin3 == 1 and fin4 == 1:
				act1 = 0
				act2 = 0
				act3 = 0
				act4 = 0
				fin1 = 0
				fin2 = 0
				fin3 = 0
				fin4 = 0
				break

		#Calculo e impresion de la distacia por pantalla
		duracion_eco1 = pulso_fin1 - pulso_inicio1
		distancia_frontal = (34300 * duracion_eco1) / 2
		#print "Distancia frontal: %.2f cm" % distancia_frontal

		duracion_eco2 = pulso_fin2 - pulso_inicio2
		distancia_izqda = (34300 * duracion_eco2) / 2
		#print "Distancia izquierda: %.2f cm" % distancia_izqda

		duracion_eco3 = pulso_fin3 - pulso_inicio3
		distancia_dcha = (34300 * duracion_eco3) / 2
		#print "Distancia derecha: %.2f cm" % distancia_dcha

		#Calculo e impresion de la distacia por pantalla
		duracion_eco4 = pulso_fin4 - pulso_inicio4
		distancia_trasera = (34300 * duracion_eco4) / 2
		#print "Distancia trasera: %.2f cm" % distancia_trasera

		#Puesta a 0 del trigger para la siguiente medida
		GPIO.output(TRIG, GPIO.LOW)


		#Deteccion de OBSTACULO
		#FRONTAL
		if sent > 0:
			if distancia_frontal < 5:
				aviso_f = 2
			elif distancia_frontal < 25:
				aviso_f = 1
			else:
				aviso_f = 0

		#TRASERO
		elif sent < 0:	
			if distancia_trasera < 5:
				aviso_t = 2
			elif distancia_trasera < 25:
				aviso_t = 1
			else:
				aviso_t = 0

		else:
			aviso_f = 0
			aviso_t = 0


		#Deteccion de APARCAMIENTO y deteccion de OBSTACULO LATERAL
		#A la derecha
		if distancia_dcha < 25:
			tiempo_inicio1 = time.time()
			hueco_dcha = 0
			aviso_d = 1
		else:
			aviso_d = 0
			tiempo_fin1 = time.time()
			#Obtencion del valor absoluto de la velocidad
			x_dcha = vel * (tiempo_fin1 - tiempo_inicio1)
			if x_dcha > 50:
				hueco_dcha = 1
				print "Hueco a la derecha"

		#A la izquierda
		if distancia_izqda < 25:
			tiempo_inicio2 = time.time()
			hueco_izqda = 0
			aviso_i = 1
		else:
			aviso_i = 1
			tiempo_fin2 = time.time()
			#Obtencion del valor absoluto de la velocidad
			x_izqda = vel * (tiempo_fin2 - tiempo_inicio2)
			if x_izqda > 50:
				hueco_izqda = 1
				print "Hueco a la izquierda"


		#BLUETOOTH
		#Lectura por puerto serie e impresion de la letra por pantalla
		motor = ser.readline()
		#print "Estado del motor: %s" % motor

		#Ejecucion de la funcion de MOTOR correspondiente a la letra
		if signal == 0:
			#Boton de STOP o deteccion de obstaculo frontal o trasero a menos de 10 cm
			if motor == "s" or aviso_f == 2 or aviso_t == 2:
				vel, vel_a, vel_b, sent = f_stop()

			elif motor == "f":
				vel, vel_a, vel_b, sent = f_fwrd()

			elif motor == "b":
				vel, vel_a, vel_b, sent = f_back()

			elif motor == "fl":
				vel, vel_a, vel_b, sent = f_fwrdleft()

			elif motor == "fr":
				vel, vel_a, vel_b, sent = f_fwrdright()

			elif motor == "bl":
				vel, vel_a, vel_b, sent = f_backleft()

			elif motor == "br":
				vel, vel_a, vel_b, sent = f_backright()
	
			elif motor == "l":
				vel, vel_a, vel_b, sent = f_left()

			elif motor == "r":
				vel, vel_a, vel_b, sent = f_right()

			#Modo PARKING (izquierda o derecha)
			elif motor == "p":
				if hueco_izqda == 1:
					parking("i")
				elif hueco_dcha == 1:
					parking("d")
				else:
					f_stop()
				        

		#WEBCAM
		_, picture = captura.read()
		hsv = cv2.cvtColor(picture, cv2.COLOR_BGR2HSV)
				
		#Máscaras de los colores
		mask_green = cv2.inRange(hsv, verde_bajos, verde_altos)
		mascara_naranja = cv2.inRange(hsv, naranja_bajos, naranja_altos)
		
		#Filtramos el ruido
		kernel = np.ones((6,6),np.uint8)
		mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
		mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
		mascara_naranja = cv2.morphologyEx(mascara_naranja, cv2.MORPH_CLOSE, kernel)
		mascara_naranja = cv2.morphologyEx(mascara_naranja, cv2.MORPH_OPEN, kernel)
		
		#Cálculo de áreas de los diferentes colores detectados
		moments_g = cv2.moments(mask_green)
		area_verde = moments_g['m00']
		moments_b = cv2.moments(mascara_naranja)
		area_naranja = moments_b['m00']
		
		if(area_verde > area and area_verde > area_naranja):
			blur=cv2.GaussianBlur(mask_green,(5,5),0)
			edges = cv2.Canny(mask_green,1,2)
			contours,hier= cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			areas = [cv2.contourArea(c) for c in contours]
			i = 0
			for extension in areas:
				if(extension > 300):
					actual = contours[i]
					approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
					if len(approx) == 4:   #Número de vértices (4=cuadrado)
						cont_cd_verde = cont_cd_verde +1
						cont_tr_verde = 0
						if(cont_cd_verde > 3):
							detec_cd_verde = 1
					elif len(approx) == 3:
						cont_tr_verde = cont_tr_verde + 1
						cont_cd_verde = 0
						if(cont_tr_verde > 3):
							detec_tr_verde = 1
					else:
						cont_cd_verde = 0
						cont_tr_verde = 0
						detec_tr_verde = 0
						detec_cd_verde = 0
				else:
					  cont_cd_verde = 0
					  cont_tr_verde = 0
					  detec_tr_verde = 0
					  detec_cd_verde = 0
			i = i + 1

		elif (area_naranja > area and area_naranja > area_verde): # and area_verde < area):
			blur=cv2.GaussianBlur(mascara_naranja,(5,5),0)
			edges = cv2.Canny(mascara_naranja,1,2)
			contours,hier= cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			areas = [cv2.contourArea(c) for c in contours]
			i = 0
			for extension in areas:
				if (extension > 300):
					actual = contours[i]
					approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
					if len(approx) == 4:   #Número de vértices (4=cuadrado)
						cont_cd_naranja = cont_cd_naranja +1
						cont_tr_naranja = 0
						if(cont_cd_naranja > 3):
							detec_cd_naranja = 1
					elif len(approx) == 3:
						cont_tr_naranja=cont_tr_naranja + 1
						cont_cd_naranja = 0
						if(cont_tr_naranja > 3):
							detec_tr_naranja = 1
					else:
						cont_cd_naranja = 0
						cont_tr_naranja = 0
						detec_tr_naranja = 0
						detec_cd_naranja = 0
				else:
					cont_tr_naranja = 0
					cont_cd_naranja = 0
					detec_tr_naranja = 0
					detec_cd_naranja = 0
			i = i + 1
			
		elif(area_naranja > area and area_verde > area):
			cont_tr_naranja = 0
			cont_cd_naranja = 0
			cont_tr_verde = 0
			cont_cd_verde = 0
			detec_tr_naranja = 0
			detec_cd_naranja = 0
			detec_tr_verde = 0
			detec_cd_verde = 0

		elif(area_naranja < area and area_verde < area):
			cont_tr_naranja = 0
			cont_cd_naranja = 0
			cont_tr_verde = 0
			cont_cd_verde = 0
			detec_tr_naranja = 0
			detec_cd_naranja = 0
			detec_tr_verde = 0
			detec_cd_verde = 0

		if (detec_cd_naranja == 1):
			print("Detectado cuadrado naranja")
		if (detec_cd_verde == 1):
			print("Detectado cuadrado verde")
		if (detec_tr_verde == 1):
			print("Detectado triángulo verde")
		if (detec_tr_naranja == 1):
			print("Detectado triángulo naranja")

		#Reaccion del coche a las SEÑALES
		#Cuadrado naranja (SEMAFORO ROJO)
		if (detec_cd_naranja == 1):
			signal = 1
			stop_inicio = 0
			vel, vel_a, vel_b, sent = f_stop()
		
		#Cuadrado verde (STOP)
		elif (detec_cd_verde == 1):
			signal = 1
			cruce = 1
			if stop_inicio == 0:
				stop_inicio = time.time()
			elif (tiempo - stop_inicio) < 5:
				vel, vel_a, vel_b, sent = f_stop()
			else:
				signal = 0
				aviso_f = 1

		#Tringulo naranja (LIMITE de VELOCIDAD)
		elif (detec_tr_naranja == 1):
			aviso_f = 1
			signal = 0
			stop_inicio = 0

		#Triangulo verde (LUCES)
		elif (detec_tr_verde == 1):
			signal = 0
			stop_inicio = 0
			if (luces == 0 and (tiempo - luces_inicio) > 5):
				luces = 1
				luces_inicio = time.time()
			elif (tiempo - luces_inicio) > 5:
				luces = 0
				luces_inicio = time.time()

		else:
			signal = 0
			stop_inicio = 0


		#REDUCCION de VELOCIDAD por OBSTACULO
		if aviso_f == 1 or aviso_t == 1:
                        if vel_a != 0:
                                pwm_a.ChangeDutyCycle(vel_a - 20)
                        if vel_b != 0:
                                pwm_b.ChangeDutyCycle(vel_b - 20)
			vel = (vel_a - 20 + vel_b - 20) / 2

		#FRENADO por CRUCE y OBSTACULO LATERAL
		if cruce == 1 and (aviso_d == 1 or aviso_i == 1):
			vel, vel_a, vel_b, sent = f_stop()
		elif sent == 1:
			cruce = 0
			
			
		#Tiempo de retardo entre ejecucion y ejecucion
		time.sleep(0.1)

#Funciones a realizar cuando se interrumpe la ejecucion
except KeyboardInterrupt:
	pwm_a.stop()
	pwm_b.stop()
	GPIO.cleanup()
	os.system('clear')
	ser.close()
	exit()
