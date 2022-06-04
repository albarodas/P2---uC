# Programa base para utilizar interfaz gráfica diseñada en QtDesigner

import sys #Variables del sistema
from PyQt5 import uic #Para conectarle las funciones
from PyQt5.QtWidgets import QMainWindow, QApplication #Obtener funcionalidades de la interfaz grafica
import serial

#VARIABLES
S1 = 0 
S2 = 0 
S3 = 0 
S4 = 0

class App(QMainWindow):
	def __init__(self): #Inicializa la clase
		super().__init__() #Inicia el contructor de la herencia del QMainWindow
		uic.loadUi("Interfaz.ui", self)	#Cargar el componente de la interfaz grafica
		self.ser = serial.Serial(port='COM3', baudrate=9600, timeout = 1.0)
		self.ser.close()
		self.Servo1.sliderReleased.connect(self.get_value_Servo1)
		self.Servo2.sliderReleased.connect(self.get_value_Servo2)
		self.Servo3.sliderReleased.connect(self.get_value_Servo3)
		self.Servo4.sliderReleased.connect(self.get_value_Servo4)

	def get_value_Servo1(self): #Vamos a usar el objeto que se inicializo
		S1 = self.Servo1.value()
		Servo_1 = int(S1)
		self.ser.open()
		self.ser.write(chr(Servo_1).encode())
		self.ser.close()
		self.lbl_Servo1.setText(str(self.Servo1.value()))
		print(chr(Servo_1).encode())

	def get_value_Servo2(self): #Vamos a usar el objeto que se inicializo
		S2 = self.Servo2.value()
		Servo_2 = int(S2 + 64)
		self.ser.open()
		self.ser.write(chr(Servo_2).encode())
		self.ser.close()
		self.lbl_Servo2.setText(str(self.Servo2.value()))
		print(chr(Servo_2).encode())

	def get_value_Servo3(self): #Vamos a usar el objeto que se inicializo
		S3 = self.Servo3.value()
		Servo_3 = int(S3 + 128)
		self.ser.open()
		self.ser.write(chr(Servo_3).encode("latin-1"))
		self.ser.close()
		self.lbl_Servo3.setText(str(self.Servo3.value()))
		print(chr(Servo_3).encode("latin-1"))

	def get_value_Servo4(self): #Vamos a usar el objeto que se inicializo
		S4 = self.Servo4.value()
		Servo_4 = int(S4 + 192) 
		self.ser.open()
		self.ser.write(chr(Servo_4).encode("latin-1"))
		self.ser.close()
		self.lbl_Servo4.setText(str(self.Servo4.value()))
		print(chr(Servo_4).encode("latin-1"))

if __name__ == '__main__':
	app = QApplication(sys.argv)
	GUI = App()
	GUI.show()
	sys.exit(app.exec_())