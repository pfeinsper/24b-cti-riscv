
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
        ser = serial.Serial(port, 19200, timeout=0)
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

        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(10)

        label_input_layout = QHBoxLayout()
        label_input_layout.setSpacing(5)

        speed_label = QLabel("Velocidade desejada (RPM):")
        speed_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        label_input_layout.addWidget(speed_label)

        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("Insira a velocidade")
        self.speed_input.setFixedWidth(100)
        label_input_layout.addWidget(self.speed_input)

        controls_layout.addLayout(label_input_layout)

        self.send_speed_button = QPushButton("Enviar")
        self.send_speed_button.setObjectName("send-speed-button")
        self.send_speed_button.setFixedSize(140, 50)
        self.send_speed_button.clicked.connect(self.set_target_speed)
        controls_layout.addWidget(self.send_speed_button)

        controls_layout.addStretch()

        self.stop_button = QPushButton("Parar")
        self.stop_button.setObjectName("stop-button")
        self.stop_button.setFixedSize(140, 50)
        self.stop_button.clicked.connect(self.stop_motor)
        controls_layout.addWidget(self.stop_button)

        self.clear_button = QPushButton("Limpar")
        self.clear_button.setObjectName("clear-button")
        self.clear_button.setFixedSize(140, 50)
        self.clear_button.clicked.connect(self.clear_terminal)
        controls_layout.addWidget(self.clear_button)

        motor_layout.addLayout(controls_layout)
        motor_group.setLayout(motor_layout)
        main_layout.addWidget(motor_group)

        # Graphs Layout
        graphs_layout = QHBoxLayout()
        graphs_layout.setSpacing(15)

        # First plot (Motor Speed)
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

        # Second plot (Raw Data)
        self.plot_widget_second = pg.PlotWidget()
        self.plot_widget_second.setBackground('#ffffff')
        self.plot_widget_second.setTitle("Dados Recebidos", color="#647881", size="12pt")
        self.plot_widget_second.setLabel("left", "Valor Recebido", units="unidade", color="#647881", size="10pt")
        self.plot_widget_second.setLabel("bottom", "Tempo", units="s", color="#647881", size="10pt")
        self.plot_widget_second.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget_second.setMouseEnabled(x=False, y=False)
        self.plot_widget_second.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Configure fixed window of 8 seconds
        self.window_size = 8  # 8 seconds window
        self.plot_widget_second.setXRange(0, self.window_size)
        self.plot_widget_second.enableAutoRange(axis='y')

        pen_second = pg.mkPen(color="#ef8327", width=2)
        self.data_second = []
        self.time_second = []
        self.time_counter_second = 0

        self.curve_second = self.plot_widget_second.plot(
            self.time_second,
            self.data_second,
            pen=pen_second,
            symbol='o',
        )

        graphs_layout.addWidget(self.plot_widget_second)

        main_layout.addLayout(graphs_layout)

        # Terminal Group
        terminal_group = QGroupBox("Terminal")
        terminal_layout = QVBoxLayout()

        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        terminal_layout.addWidget(self.terminal)
        terminal_group.setLayout(terminal_layout)
        main_layout.addWidget(terminal_group)

        self.setLayout(main_layout)

        # Initialize timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(50)

        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_uart_data)

        # Initialize variables
        self.motor_running = False
        self.current_speed = 0
        self.target_speed = 0
        self.ser = None

    def clear_terminal(self):
        self.terminal.clear()
        self.data_second = []
        self.time_second = []
        self.time_counter_second = 0
        self.curve_second.setData(self.time_second, self.data_second)
        self.plot_widget_second.setXRange(0, self.window_size)

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

    def set_target_speed(self):
        text = self.speed_input.text().strip()  # Remove espaços extras
        if text:
            if self.ser and self.ser.is_open:
                command = f"{text}\n"  # Adiciona nova linha no final
                self.ser.write(command.encode('utf-8'))
                self.terminal.append(f"Comando enviado: {text}")
                self.speed_input.clear()  # Limpa o campo de input após enviar
                self.terminal.ensureCursorVisible()
        else:
            self.terminal.append("Por favor, insira um comando.")
            self.terminal.ensureCursorVisible()


    def stop_motor(self):
        self.motor_running = False
        self.target_speed = 0
        self.terminal.append("Motor parado.")
        self.terminal.ensureCursorVisible()
        self.curve_target.setData(np.full(100, self.target_speed))
        if self.ser and self.ser.is_open:
            command = "STOP_MOTOR\n"
            self.ser.write(command.encode('utf-8'))

    def update_graphs(self):
        pass
        # if self.motor_running:
        #     if self.current_speed < self.target_speed:
        #         self.current_speed += 3500
        #         if self.current_speed > self.target_speed:
        #             self.current_speed = self.target_speed
        #     elif self.current_speed > self.target_speed:
        #         self.current_speed -= 3500
        #         if self.current_speed < self.target_speed:
        #             self.current_speed = self.target_speed
        # else:
        #     if self.current_speed > 0:
        #         self.current_speed -= 1500
        #         if self.current_speed < 0:
        #             self.current_speed = 0

        # self.data_current = np.roll(self.data_current, -1)
        # self.data_current[-1] = self.current_speed
        # self.curve_current.setData(self.data_current)

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
                            
                            # Try to parse "speed: value" format
                            speed_match = re.search(r'speed:\s*(\d+)', line, re.IGNORECASE)
                            if speed_match:
                                try:
                                    speed_value = float(speed_match.group(1))
                                    self.current_speed = speed_value
                                    
                                    # Update second graph with sliding window
                                    current_time = self.time_counter_second / 10.0  # Convert to seconds
                                    self.data_second.append(speed_value)
                                    self.time_second.append(current_time)
                                    
                                    # Remove data points outside the window
                                    while len(self.time_second) > 0 and current_time - self.time_second[0] > self.window_size:
                                        self.time_second.pop(0)
                                        self.data_second.pop(0)
                                    
                                    # Update plot with sliding window
                                    self.curve_second.setData(self.time_second, self.data_second)
                                    self.plot_widget_second.setXRange(
                                        max(0, current_time - self.window_size),
                                        current_time
                                    )
                                    
                                    self.time_counter_second += 1
                                    self.terminal.append(f"Velocidade atualizada: {speed_value} RPM")
                                    self.terminal.ensureCursorVisible()
                                except ValueError:
                                    pass

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
