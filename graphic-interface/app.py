import sys
import os
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QLineEdit, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QIntValidator
import pyqtgraph as pg
import serial
import re

def connect_uart(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0)
        print(f"Conectado ao dispositivo UART na porta {port}.")
        return ser
    except serial.SerialException as e:
        print(f"Erro ao conectar ao UART: {e}")
        return None

class MotorControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Interface")
        self.showMaximized()

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        top_layout = QHBoxLayout()

        uart_group = QGroupBox("Conexão UART")
        uart_layout = QHBoxLayout()
        uart_layout.setSpacing(10)

        self.port_input = QLineEdit(self)
        self.port_input.setPlaceholderText("Ex: /dev/ttyACM0")
        self.port_input.setText('/dev/COMA')
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

        # Grupo de Controle do Motor
        motor_group = QGroupBox("Controle do Motor")
        motor_layout = QVBoxLayout()
        motor_layout.setSpacing(15)

        # Novo layout horizontal para todos os controles
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(10)

        # Sub-layout para o label e input
        label_input_layout = QHBoxLayout()
        label_input_layout.setSpacing(5)

        speed_label = QLabel("Velocidade desejada (RPM):")
        speed_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        label_input_layout.addWidget(speed_label)

        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Insira a velocidade")
        self.speed_input.setFixedWidth(100)
        self.speed_input.setValidator(QIntValidator(0, 21000))
        label_input_layout.addWidget(self.speed_input)

        controls_layout.addLayout(label_input_layout)

        # Botão "Enviar" ao lado do input
        self.send_speed_button = QPushButton("Enviar")
        self.send_speed_button.setObjectName("send-speed-button")
        self.send_speed_button.setFixedSize(140, 50)
        self.send_speed_button.clicked.connect(self.set_target_speed)
        controls_layout.addWidget(self.send_speed_button)

        # Espaçamento flexível para empurrar os próximos botões para a direita
        controls_layout.addStretch()

        # Botão "Parar"
        self.stop_button = QPushButton("Parar")
        self.stop_button.setObjectName("stop-button")
        self.stop_button.setFixedSize(140, 50)
        self.stop_button.clicked.connect(self.stop_motor)
        controls_layout.addWidget(self.stop_button)

        # Botão "Limpar"
        self.clear_button = QPushButton("Limpar")
        self.clear_button.setObjectName("clear-button")
        self.clear_button.setFixedSize(140, 50)
        self.clear_button.clicked.connect(self.clear_terminal)
        controls_layout.addWidget(self.clear_button)

        motor_layout.addLayout(controls_layout)
        motor_group.setLayout(motor_layout)
        main_layout.addWidget(motor_group)

        graphs_layout = QHBoxLayout()
        graphs_layout.setSpacing(15)

        # Gráfico de Velocidade do Motor
        self.plot_widget_current = pg.PlotWidget()
        self.plot_widget_current.setBackground('#ffffff')
        self.plot_widget_current.setTitle("Velocidade do Motor", color="#647881", size="12pt")
        self.plot_widget_current.setLabel("left", "Velocidade", units="RPM", color="#647881", size="10pt")
        self.plot_widget_current.setLabel("bottom", "Tempo", units="s", color="#647881", size="10pt")
        self.plot_widget_current.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget_current.setYRange(0, 21000, padding=0)
        self.plot_widget_current.setXRange(0, 100, padding=0)
        self.plot_widget_current.setMouseEnabled(x=False, y=False)
        self.plot_widget_current.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        pen_current = pg.mkPen(color="#ef8327", width=2)
        pen_target = pg.mkPen(color="#647881", width=2, style=Qt.DashLine)

        self.data_current = np.zeros(100)
        self.curve_current = self.plot_widget_current.plot(self.data_current, pen=pen_current, name="Velocidade Atual")

        self.data_target = np.full(100, 0)
        self.curve_target = self.plot_widget_current.plot(self.data_target, pen=pen_target, name="Velocidade Target")

        graphs_layout.addWidget(self.plot_widget_current)

        # Segundo Gráfico
        self.plot_widget_second = pg.PlotWidget()
        self.plot_widget_second.setBackground('#ffffff')
        self.plot_widget_second.setTitle("Segundo Gráfico", color="#647881", size="12pt")
        self.plot_widget_second.setLabel("left", "Y", units="unidade", color="#647881", size="10pt")
        self.plot_widget_second.setLabel("bottom", "X", units="unidade", color="#647881", size="10pt")
        self.plot_widget_second.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget_second.setYRange(0, 24000, padding=0)
        self.plot_widget_second.setXRange(0, 100, padding=0)
        self.plot_widget_second.setMouseEnabled(x=False, y=False)
        self.plot_widget_second.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        pen_second = pg.mkPen(color="#ef8327", width=2)

        self.data_second = np.zeros(100)
        self.curve_second = self.plot_widget_second.plot(self.data_second, pen=pen_second)

        graphs_layout.addWidget(self.plot_widget_second)

        main_layout.addLayout(graphs_layout)

        # Grupo do Terminal
        terminal_group = QGroupBox("Terminal")
        terminal_layout = QVBoxLayout()

        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        terminal_layout.addWidget(self.terminal)
        terminal_group.setLayout(terminal_layout)
        main_layout.addWidget(terminal_group)

        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(50)

        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_uart_data)

        self.motor_running = False
        self.current_speed = 0
        self.target_speed = 0

        self.ser = None

    def clear_terminal(self):
        self.terminal.clear()

    def connect_to_uart(self):
        port = self.port_input.text()
        if port:
            self.ser = connect_uart(port)
            if self.ser and self.ser.is_open:
                self.set_connection_status(active=True)
                self.read_timer.start(100)
                self.terminal.append(f"Conectado à porta {port}.")
            else:
                self.set_connection_status(active=False)
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

    def set_target_speed(self):
        text = self.speed_input.text()
        try:
            speed = float(text)
            self.target_speed = speed
            self.motor_running = True
            self.terminal.append(f"Velocidade target definida para: {self.target_speed} RPM.")
            self.terminal.ensureCursorVisible()
            self.curve_target.setData(np.full(100, self.target_speed))
            if self.ser and self.ser.is_open:
                command = f"SET_SPEED {self.target_speed}\n"
                self.ser.write(command.encode('utf-8'))
        except ValueError:
            self.terminal.append("Por favor, insira um valor numérico válido para a velocidade.")
            self.terminal.ensureCursorVisible()

    def stop_motor(self):
        self.motor_running = False
        self.target_speed = 0
        self.terminal.append("Motor parado.")
        self.terminal.ensureCursorVisible()
        self.curve_target.setData(np.full(100, self.target_speed))
        if self.ser and self.ser.is_open:
            command = f"STOP_MOTOR\n"
            self.ser.write(command.encode('utf-8'))

    def update_graphs(self):
        if self.motor_running:
            if self.current_speed < self.target_speed:
                self.current_speed += 3500
                if self.current_speed > self.target_speed:
                    self.current_speed = self.target_speed
            elif self.current_speed > self.target_speed:
                self.current_speed -= 3500
                if self.current_speed < self.target_speed:
                    self.current_speed = self.target_speed
        else:
            if self.current_speed > 0:
                self.current_speed -= 1500
                if self.current_speed < 0:
                    self.current_speed = 0

        self.data_current = np.roll(self.data_current, -1)
        self.data_current[-1] = self.current_speed
        self.curve_current.setData(self.data_current)

    def read_uart_data(self):
            if self.ser and self.ser.is_open:
                try:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        lines = data.decode('utf-8', errors='ignore').split('\n')
                        for line in lines:
                            line = line.strip()
                            if line:
                                self.terminal.append(line)
                                self.terminal.ensureCursorVisible()
                                print(f"Dados recebidos: {line}")

                                try:
                                    value = float(line)
                                    self.data_second = np.roll(self.data_second, -1)
                                    self.data_second[-1] = value
                                    self.curve_second.setData(self.data_second)
                                    self.plot_widget_second.autoRange()
                                    self.terminal.append(f"Valor recebido: {value}")
                                    self.terminal.ensureCursorVisible()
                                except ValueError:
                                    match = re.search(r'velocidade:\s*(\d+)', line)
                                    if match:
                                        self.current_speed = float(match.group(1))

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
