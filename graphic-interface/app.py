import sys
import os
import numpy as np
from PyQt5.QtWidgets import (
	QApplication, QWidget, QVBoxLayout, QHBoxLayout,
	QPushButton, QLabel, QTextEdit, QLineEdit, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import QTimer, Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon
import pyqtgraph as pg
import serial
import re
import time


def connect_uart(port):
	try:
		ser = serial.Serial(port, 9600, timeout=1)
		print(f"Conectado ao dispositivo UART na porta {port}.")
		return ser
	except serial.SerialException as e:
		print(f"Erro ao conectar ao UART: {e}")
		return None


class MotorControlApp(QWidget):
	def __init__(self):
		super().__init__()
		self.uart_buffer = ''  # Buffer para armazenar dados UART incompletos
		self.setWindowTitle("Motor Control Interface")
		self.showMaximized()
		

		main_layout = QVBoxLayout()
		main_layout.setContentsMargins(20, 20, 20, 20)
		main_layout.setSpacing(15)


		# Top layout with UART connection and logo
		top_layout = QHBoxLayout()
		uart_group = QGroupBox("Conexão UART")
		uart_layout = QHBoxLayout()
		uart_layout.setSpacing(10)


		self.port_input = QLineEdit(self)
		self.port_input.setPlaceholderText("Ex: /dev/ttyACM0")
		self.port_input.setText('/dev/ttyACM0')
		self.port_input.setFixedWidth(200)
		uart_layout.addWidget(self.port_input)


		self.connect_button = QPushButton("Conectar")
		self.connect_button.setObjectName("connect-button")
		self.connect_button.setFixedWidth(120)
		self.connect_button.clicked.connect(self.connect_to_uart)
		uart_layout.addWidget(self.connect_button)


		self.connection_status = QLabel("Inativa")
		self.connection_status.setAlignment(Qt.AlignCenter)
		self.connection_status.setFixedWidth(120)
		self.connection_status.setObjectName("connection-status-inactive")
		uart_layout.addWidget(self.connection_status)


		uart_group.setLayout(uart_layout)
		top_layout.addWidget(uart_group, alignment=Qt.AlignLeft)
		top_layout.addStretch()


		# Logo
		logo_label = QLabel()
		logo_label.setObjectName("logo-label")
		logo_path = os.path.join(os.path.dirname(__file__), "cti_renato_archer_logo.jpeg")
		if os.path.exists(logo_path):
			logo_pixmap = QPixmap(logo_path)
			logo_pixmap = logo_pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
			logo_label.setPixmap(logo_pixmap)
		else:
			logo_label.setText("Logo")
		top_layout.addWidget(logo_label, alignment=Qt.AlignRight)
		main_layout.addLayout(top_layout)


		# Motor Control Group
		motor_group = QGroupBox("Controle do Motor")
		motor_layout = QVBoxLayout()
		motor_layout.setSpacing(15)


		# First row: Command input and buttons
		first_row = QHBoxLayout()
		first_row.setSpacing(10)


		command_layout = QHBoxLayout()
		command_layout.setSpacing(5)


		speed_label = QLabel("Comando para enviar:")
		command_layout.addWidget(speed_label)


		self.speed_input = QLineEdit()
		self.speed_input.setObjectName("command-input")
		self.speed_input.setPlaceholderText("Insira o comando")
		command_layout.addWidget(self.speed_input)


		first_row.addLayout(command_layout)


		self.send_speed_button = QPushButton("Enviar")
		self.send_speed_button.setObjectName("send-speed-button")
		self.send_speed_button.clicked.connect(self.send_command)
		first_row.addWidget(self.send_speed_button)


		self.clear_button = QPushButton("Limpar")
		self.clear_button.setObjectName("clear-button")
		self.clear_button.clicked.connect(self.clear_terminal)
		first_row.addWidget(self.clear_button)


		self.power_button = QPushButton("ON/OFF")
		self.power_button.setObjectName("power-button")
		self.power_button.clicked.connect(self.power_motor)
		first_row.addWidget(self.power_button)


		first_row.addStretch()
		motor_layout.addLayout(first_row)


		# Second row: ki, kp inputs, and direction buttons
		second_row = QHBoxLayout()
		second_row.setObjectName("parameters-row")


		ki_layout = QHBoxLayout()
		ki_label = QLabel("ki:")
		self.ki_input = QLineEdit()
		self.ki_input.setObjectName("ki-input")
		self.ki_input.setPlaceholderText("Enter ki value")
		self.send_ki_button = QPushButton("Set ki")
		self.send_ki_button.setObjectName("send-speed-button")  # Using same style as other buttons
		self.send_ki_button.clicked.connect(self.send_ki_value)
		self.send_ki_button.setFixedSize(140, 50)  # Match other buttons size
		ki_layout.addWidget(ki_label)
		ki_layout.addWidget(self.ki_input)
		ki_layout.addWidget(self.send_ki_button)
		second_row.addLayout(ki_layout)


		# kp section:
		kp_layout = QHBoxLayout()
		kp_label = QLabel("kp (%):")
		self.kp_input = QLineEdit()
		self.kp_input.setObjectName("kp-input")
		self.kp_input.setPlaceholderText("Enter kp value")
		self.send_kp_button = QPushButton("Set kp")
		self.send_kp_button.setObjectName("send-speed-button")
		self.send_kp_button.setFixedSize(140, 50)
		self.send_kp_button.clicked.connect(self.send_kp_value)
		kp_layout.addWidget(kp_label)
		kp_layout.addWidget(self.kp_input)
		kp_layout.addWidget(self.send_kp_button)
		second_row.addLayout(kp_layout)
		second_row.addStretch()


		direction_layout = QHBoxLayout()
		direction_layout.setObjectName("direction-buttons")
		
		self.clockwise_button = QPushButton()
		self.clockwise_button.setObjectName("clockwise-button")
		self.clockwise_button.setFixedSize(50, 50)
		clockwise_icon_path = os.path.join(os.path.dirname(__file__), "rotate-right.png")
		if os.path.exists(clockwise_icon_path):
			self.clockwise_button.setIcon(QIcon(clockwise_icon_path))
			self.clockwise_button.setIconSize(QSize(50, 50))
		else:
			self.clockwise_button.setText("CW")
		self.clockwise_button.clicked.connect(self.set_clockwise_direction)


		# Counter-clockwise button
		self.counterclockwise_button = QPushButton()
		self.counterclockwise_button.setObjectName("counterclockwise-button")
		self.counterclockwise_button.setFixedSize(50, 50)
		counterclockwise_icon_path = os.path.join(os.path.dirname(__file__), "ccw.png")
		if os.path.exists(counterclockwise_icon_path):
			self.counterclockwise_button.setIcon(QIcon(counterclockwise_icon_path))
			self.counterclockwise_button.setIconSize(QSize(50, 50))
		else:
			self.counterclockwise_button.setText("CCW")
		self.counterclockwise_button.clicked.connect(self.set_counterclockwise_direction)


		direction_layout.addWidget(self.clockwise_button)
		direction_layout.addWidget(self.counterclockwise_button)
		second_row.addLayout(direction_layout)


		motor_layout.addLayout(second_row)
		motor_group.setLayout(motor_layout)
		main_layout.addWidget(motor_group)


		# Graph
		graphs_layout = QHBoxLayout()
		graphs_layout.setSpacing(15)


		self.plot_widget_second = pg.PlotWidget()
		self.plot_widget_second.setBackground('#ffffff')
		self.plot_widget_second.setTitle("Dados Recebidos", color="#647881", size="12pt")
		self.plot_widget_second.setLabel("left", "Velocidade", units="RPM", color="#647881", size="10pt")
		self.plot_widget_second.setLabel("bottom", "Tempo", units="s", color="#647881", size="10pt")
		self.plot_widget_second.showGrid(x=True, y=True, alpha=0.3)
		self.plot_widget_second.setMouseEnabled(x=False, y=False)
		self.plot_widget_second.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
		self.plot_widget_second.enableAutoRange('x', True)
		self.plot_widget_second.setYRange(0, 18000)  


		symbol_pen = pg.mkPen(color="#ef8327")
		symbol_brush = pg.mkBrush(color="#ef8327")


		self.data_second = []
		self.time_second = []


		self.curve_second = self.plot_widget_second.plot(
			self.time_second,
			self.data_second,
			pen=None,
			symbol='o',
			symbolPen=symbol_pen,
			symbolBrush=symbol_brush
		)


		graphs_layout.addWidget(self.plot_widget_second)
		main_layout.addLayout(graphs_layout)


		# Terminal
		terminal_group = QGroupBox("Terminal")
		terminal_layout = QVBoxLayout()


		self.terminal = QTextEdit()
		self.terminal.setReadOnly(True)
		self.terminal.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
		terminal_layout.addWidget(self.terminal)
		terminal_group.setLayout(terminal_layout)
		main_layout.addWidget(terminal_group)


		self.setLayout(main_layout)


		self.read_timer = QTimer()
		self.read_timer.timeout.connect(self.read_uart_data)


		self.ser = None


	def clear_terminal(self):
		self.terminal.clear()


	def power_motor(self):
		command = "power:"
		self.terminal.append("Power command sent.")
		self.send_uart_command(command)


	def connect_to_uart(self):
		port = self.port_input.text()
		if port:
			self.ser = connect_uart(port)
			if self.ser and self.ser.is_open:
				self.set_connection_status(active=True)
				self.read_timer.start(2)
				self.terminal.append(f"Conectado à porta {port}.")
			else:
				self.set_connection_status(active=False)
				self.terminal.append(f"Não foi possível conectar à porta {port}.")
		else:
			self.terminal.append("Por favor, insira o caminho do dispositivo UART.")


	def set_connection_status(self, active):
		if active:
			self.connection_status.setText("Ativa")
			self.connection_status.setObjectName("connection-status-active")
		else:
			self.connection_status.setText("Inativa")
			self.connection_status.setObjectName("connection-status-inactive")


		self.connection_status.style().unpolish(self.connection_status)
		self.connection_status.style().polish(self.connection_status)
		self.connection_status.update()


	def send_command(self):
		command = self.speed_input.text()
		if command:
			command_to_send = f"speed:{command}"
			self.send_uart_command(command_to_send)
			self.terminal.append(f"Comando enviado: {command_to_send}")
			self.terminal.ensureCursorVisible()
		else:
			self.terminal.append("Por favor, insira um comando para enviar.")
			self.terminal.ensureCursorVisible()


	def send_uart_command(self, command):
		if self.ser and self.ser.is_open:
			command_to_send = f"{command}\r\n"
			self.ser.write(command_to_send.encode('utf-8'))
		else:
			self.terminal.append("UART connection is not open.")
			self.terminal.ensureCursorVisible()


	def set_clockwise_direction(self):
		command = "dir:1"
		self.send_uart_command(command)
		self.terminal.append("Motor direction set to clockwise.")
		self.terminal.ensureCursorVisible()


	def set_counterclockwise_direction(self):
		command = "dir:0"
		self.send_uart_command(command)
		self.terminal.append("Motor direction set to counter-clockwise.")
		self.terminal.ensureCursorVisible()


	def send_ki_value(self):
		ki_input = self.ki_input.text()
		if ki_input:
			try:
				ki_float = float(ki_input)
				ki_value = ki_float * 1e8
				ki_value = int(ki_value)
				command = f"ki:{ki_value}"
				self.send_uart_command(command)
				self.terminal.append(f"ki set to {ki_value}.")
			except ValueError:
				self.terminal.append("Invalid ki value.")
		else:
			self.terminal.append("Please enter a ki value.")
		self.terminal.ensureCursorVisible()


	def send_kp_value(self):
		kp_input = self.kp_input.text()
		if kp_input:
			try:
				kp_float = float(kp_input)
				kp_value = kp_float * 1e8
				kp_value = int(kp_value)
				command = f"kp:{kp_value}"
				self.send_uart_command(command)
				self.terminal.append(f"kp set to {kp_value}%.")
			except ValueError:
				self.terminal.append("Invalid kp value.")
		else:
			self.terminal.append("Please enter a kp value.")
		self.terminal.ensureCursorVisible()


	def read_uart_data(self):
		if self.ser and self.ser.is_open:
			try:
				data = self.ser.read(self.ser.in_waiting)
				if data:
					self.uart_buffer += data.decode('utf-8', errors='ignore')
					while '?' in self.uart_buffer:
						index = self.uart_buffer.find('?')
						line = self.uart_buffer[:index]
						self.uart_buffer = self.uart_buffer[index+1:]
						line = line.strip()
						if line:
							self.terminal.append(line)
							self.terminal.ensureCursorVisible()
							try:
								value = float(line)
								timestamp = time.time()
								self.data_second.append(value)
								self.time_second.append(timestamp)


								cutoff_time = timestamp - 20
								while self.time_second and self.time_second[0] < cutoff_time:
									self.time_second.pop(0)
									self.data_second.pop(0)
								times_relative = [t - cutoff_time for t in self.time_second]
								self.curve_second.setData(times_relative, self.data_second)
								self.plot_widget_second.setXRange(0, 20)
								self.terminal.append(f"Valor recebido: {value}")
								self.terminal.ensureCursorVisible()
							except ValueError:
								match = re.search(r'Speed:\s*(\d+)', line)
								if match:
									self.current_speed = float(match.group(1))
									self.terminal.append(f"Velocidade atualizada: {self.current_speed} RPM")
									self.terminal.ensureCursorVisible()
			except serial.SerialException as e:
				print(f"Erro de leitura UART: {e}")
				self.terminal.append(f"Erro de leitura UART: {e}")
				self.terminal.ensureCursorVisible()
		else:
			self.set_connection_status(active=False)
			self.terminal.append("Conexão UART perdida.")
			self.read_timer.stop()


if __name__ == "__main__":
	app = QApplication(sys.argv)


	qss_file = os.path.join(os.path.dirname(__file__), "style.qss")
	if os.path.exists(qss_file):
		with open(qss_file, "r") as f:
			app.setStyleSheet(f.read())
	else:
		print("Arquivo de estilos 'style.qss' não encontrado.")


	window = MotorControlApp()
	window.show()
	sys.exit(app.exec_())
