import sys
import os
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QLineEdit, QGroupBox, QSizePolicy, QButtonGroup, QGridLayout, QSpacerItem
)
from PyQt5.QtCore import QTimer, Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon
import pyqtgraph as pg
import serial
import re
import time

# Definição das constantes utilizadas na função get_pi
valor_estabilizado = 16589
duty_inicial = 0
duty_final = 1
dt = 54e-3
k = valor_estabilizado / (duty_final - duty_inicial)
a = 1 / dt

PI_SQUARED = np.pi ** 2

def get_pi(up: float, ts: float) -> tuple[float, float]:
    print(up, ts)
    """
    Compute the proportional (kp) and integral (ki) gains for a control system.

    Parameters:
        up (float): A parameter, must be positive.
        ts (float): Settling time, must be positive.

    Returns:
        tuple[float, float]: ki and kp gains.
    """
    # Pre-check conditions
    if up <= 0 or ts <= 0:
        raise ValueError("Both 'up' and 'ts' must be positive.")
    if k <= a:
        raise ValueError("'k' must be greater than 'a'.")

    # Fast calculations
    log_up = np.log(up)
    num = log_up ** 2
    den = num + PI_SQUARED
    zeta = (num / den) ** 0.5  # Equivalent to np.sqrt for scalars

    if zeta == 0:
        raise ValueError("Calculation resulted in zeta=0.")

    wn = 4 / (zeta * ts)
    wn_squared = wn ** 2  # Avoid recalculating
    ki = wn_squared / (k * a)
    kp = (2 * zeta * wn - a) / (k - a)

    return ki, kp

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
        self.uart_buffer = ''
        self.setWindowTitle("Motor Control Interface")
        self.showMaximized()

        # Layout principal vertical
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)  # Margens externas
        main_layout.setSpacing(15)  # Espaçamento entre widgets

        # Top layout com conexão UART e logo
        top_layout = QHBoxLayout()
        top_layout.setContentsMargins(0, 0, 0, 0)  # Remover margens internas
        uart_group = QGroupBox("Conexão UART")
        uart_layout = QHBoxLayout()
        uart_layout.setContentsMargins(0, 0, 0, 0)  # Remover margens internas
        uart_layout.setSpacing(5)  # Espaçamento entre elementos

        # Campo de entrada para porta UART
        self.port_input = QLineEdit(self)
        self.port_input.setPlaceholderText("Ex: /dev/ttyACM0")
        self.port_input.setText('/dev/ttyACM0')
        self.port_input.setFixedWidth(200)
        self.port_input.setToolTip("Insira o caminho da porta UART")
        uart_layout.addWidget(self.port_input)

        # Botão de conectar UART
        self.connect_button = QPushButton("Conectar")
        self.connect_button.setObjectName("connect-button")
        self.connect_button.setFixedWidth(120)
        connect_icon_path = os.path.join(os.path.dirname(__file__), "connect_icon.png")
        if os.path.exists(connect_icon_path):
            self.connect_button.setIcon(QIcon(connect_icon_path))
            self.connect_button.setIconSize(QSize(20, 20))
        self.connect_button.setToolTip("Clique para conectar à porta UART")
        self.connect_button.clicked.connect(self.connect_to_uart)
        uart_layout.addWidget(self.connect_button)

        # Status da conexão UART
        self.connection_status = QLabel("Inativa")
        self.connection_status.setAlignment(Qt.AlignCenter)
        self.connection_status.setFixedWidth(120)
        self.connection_status.setObjectName("connection-status-inactive")
        uart_layout.addWidget(self.connection_status)

        uart_group.setLayout(uart_layout)
        top_layout.addWidget(uart_group, alignment=Qt.AlignLeft)  # Alinhar à esquerda

        # Adicionar logo à direita
        logo_label = QLabel()
        logo_label.setObjectName("logo-label")
        logo_path = os.path.join(os.path.dirname(__file__), "images/cti_renato_archer_logo.jpeg")
        if os.path.exists(logo_path):
            logo_pixmap = QPixmap(logo_path)
            logo_pixmap = logo_pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            logo_label.setPixmap(logo_pixmap)
        else:
            logo_label.setText("Logo")
        top_layout.addWidget(logo_label, alignment=Qt.AlignRight)  # Alinhar à direita

        main_layout.addLayout(top_layout)

        # Grupo de Controle do Motor
        motor_group = QGroupBox("Controle do Motor")
        motor_layout = QVBoxLayout()
        motor_layout.setSpacing(10)  # Espaçamento interno

        # Primeira linha: Comando e Botões
        first_row = QHBoxLayout()
        first_row.setSpacing(10)  # Espaçamento entre elementos

        # Layout para "Comando para enviar:" e input
        command_layout = QHBoxLayout()
        command_layout.setSpacing(2)  # Redução do espaçamento para aproximar elementos

        speed_label = QLabel("Comando para enviar:")
        speed_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)  # Alinhar verticalmente ao centro e à esquerda
        command_layout.addWidget(speed_label)

        self.speed_input = QLineEdit()
        self.speed_input.setObjectName("command-input")
        self.speed_input.setPlaceholderText("Insira o comando")
        self.speed_input.setFixedWidth(200)
        self.speed_input.setToolTip("Insira o comando de velocidade para enviar ao motor")
        command_layout.addWidget(self.speed_input)

        first_row.addLayout(command_layout)

        # Layout para botões "Enviar", "Limpar" e "ON/OFF"
        command_buttons_layout = QHBoxLayout()
        command_buttons_layout.setSpacing(5)  # Espaçamento entre botões

        self.send_speed_button = QPushButton("Enviar")
        self.send_speed_button.setObjectName("send-speed-button")
        send_icon_path = os.path.join(os.path.dirname(__file__), "send_icon.png")
        if os.path.exists(send_icon_path):
            self.send_speed_button.setIcon(QIcon(send_icon_path))
            self.send_speed_button.setIconSize(QSize(20, 20))
        self.send_speed_button.setToolTip("Clique para enviar o comando de velocidade")
        self.send_speed_button.setFixedWidth(100)
        self.send_speed_button.clicked.connect(self.send_command)
        command_buttons_layout.addWidget(self.send_speed_button)

        self.clear_button = QPushButton("Limpar")
        self.clear_button.setObjectName("clear-button")
        clear_icon_path = os.path.join(os.path.dirname(__file__), "clear_icon.png")
        if os.path.exists(clear_icon_path):
            self.clear_button.setIcon(QIcon(clear_icon_path))
            self.clear_button.setIconSize(QSize(20, 20))
        self.clear_button.setToolTip("Clique para limpar o terminal")
        self.clear_button.setFixedWidth(100)
        self.clear_button.clicked.connect(self.clear_terminal)
        command_buttons_layout.addWidget(self.clear_button)

        self.power_button = QPushButton("ON/OFF")
        self.power_button.setObjectName("power-button")
        power_icon_path = os.path.join(os.path.dirname(__file__), "power_icon.png")
        if os.path.exists(power_icon_path):
            self.power_button.setIcon(QIcon(power_icon_path))
            self.power_button.setIconSize(QSize(20, 20))
        self.power_button.setToolTip("Clique para ligar ou desligar o motor")
        self.power_button.setFixedWidth(100)
        self.power_button.clicked.connect(self.power_motor)
        self.motor_on = False
        self.power_button.setStyleSheet("background-color: #ef4444;")  # Define vermelho inicial
        command_buttons_layout.addWidget(self.power_button)

        first_row.addLayout(command_buttons_layout)

        # Adicionar um espaçador flexível para manter os elementos alinhados à esquerda
        first_row.addStretch()
        motor_layout.addLayout(first_row)

        # Segunda linha: up, ts, send pi e botões de direção
        second_row = QHBoxLayout()
        second_row.setSpacing(10)
        second_row.setContentsMargins(0, 0, 0, 0)  # Remover margens internas

        # Sub-layout esquerdo para up, ts e send pi
        left_layout = QGridLayout()
        left_layout.setSpacing(5)
        left_layout.setContentsMargins(0, 0, 0, 0)  # Remover margens internas

        up_label = QLabel("up (%):")
        up_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.up_input = QLineEdit()
        self.up_input.setObjectName("up-input")
        self.up_input.setPlaceholderText("Ex: 5")
        self.up_input.setFixedWidth(100)
        self.up_input.setToolTip("Insira o valor de up para calcular ki e kp")
        left_layout.addWidget(up_label, 0, 0)
        left_layout.addWidget(self.up_input, 0, 1)

        ts_label = QLabel("ts (s):")
        ts_label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.ts_input = QLineEdit()
        self.ts_input.setObjectName("ts-input")
        self.ts_input.setPlaceholderText("Ex: 0.27")
        self.ts_input.setFixedWidth(100)
        self.ts_input.setToolTip("Insira o tempo de assentamento (ts) em segundos")
        left_layout.addWidget(ts_label, 1, 0)
        left_layout.addWidget(self.ts_input, 1, 1)

        self.send_pi_button = QPushButton("Calcular e Enviar PI")
        self.send_pi_button.setObjectName("send-pi-button")
        send_pi_icon_path = os.path.join(os.path.dirname(__file__), "calculate_icon.png")
        if os.path.exists(send_pi_icon_path):
            self.send_pi_button.setIcon(QIcon(send_pi_icon_path))
            self.send_pi_button.setIconSize(QSize(20, 20))
        self.send_pi_button.setToolTip("Clique para calcular e enviar os valores de ki e kp")
        self.send_pi_button.setFixedSize(200, 40)
        self.send_pi_button.clicked.connect(self.calculate_and_send_pi)
        left_layout.addWidget(self.send_pi_button, 0, 2, 2, 1, Qt.AlignVCenter)  # Span em duas linhas

        second_row.addLayout(left_layout)

        # Adicionar um espaçador flexível para empurrar os botões de direção para a direita
        spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        second_row.addItem(spacer)

        # Sub-layout direito para botões de direção
        direction_buttons_layout = QHBoxLayout()
        direction_buttons_layout.setSpacing(5)

        self.clockwise_button = QPushButton()
        self.clockwise_button.setObjectName("clockwise-button")
        self.clockwise_button.setFixedSize(60, 60)
        self.clockwise_button.setCheckable(True)
        self.clockwise_button.setChecked(True)
        clockwise_icon_path = os.path.join(os.path.dirname(__file__), "images/rotate-right.png")
        if os.path.exists(clockwise_icon_path):
            self.clockwise_button.setIcon(QIcon(clockwise_icon_path))
            self.clockwise_button.setIconSize(QSize(40, 40))
        else:
            self.clockwise_button.setText("CW")
        self.clockwise_button.setToolTip("Clique para definir a direção como Horário")
        direction_buttons_layout.addWidget(self.clockwise_button)

        self.counterclockwise_button = QPushButton()
        self.counterclockwise_button.setObjectName("counterclockwise-button")
        self.counterclockwise_button.setFixedSize(60, 60)
        self.counterclockwise_button.setCheckable(True)
        self.counterclockwise_button.setChecked(False)
        counterclockwise_icon_path = os.path.join(os.path.dirname(__file__), "images/ccw.png")
        if os.path.exists(counterclockwise_icon_path):
            self.counterclockwise_button.setIcon(QIcon(counterclockwise_icon_path))
            self.counterclockwise_button.setIconSize(QSize(40, 40))
        else:
            self.counterclockwise_button.setText("CCW")
        self.counterclockwise_button.setToolTip("Clique para definir a direção como Anti-horário")
        direction_buttons_layout.addWidget(self.counterclockwise_button)

        second_row.addLayout(direction_buttons_layout)

        motor_layout.addLayout(second_row)

        motor_group.setLayout(motor_layout)
        main_layout.addWidget(motor_group, alignment=Qt.AlignLeft)  # Alinhar à esquerda

        # Agrupar botões de direção em um grupo exclusivo
        self.direction_group = QButtonGroup()
        self.direction_group.setExclusive(True)
        self.direction_group.addButton(self.clockwise_button)
        self.direction_group.addButton(self.counterclockwise_button)

        self.direction_group.buttonClicked.connect(self.handle_direction_change)

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
            symbolBrush=symbol_brush,
            symbolSize=8
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

        # Timer para leitura UART
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_uart_data)

        self.ser = None

    def clear_terminal(self):
        self.terminal.clear()

    def power_motor(self):
        self.motor_on = not self.motor_on
        color = "#22c55e" if self.motor_on else "#ef4444"
        self.power_button.setStyleSheet(f"background-color: {color};")
        command = "power:"
        self.terminal.append(f"Power {'ON' if self.motor_on else 'OFF'} command sent.")
        self.send_uart_command(command)

    def connect_to_uart(self):
        port = self.port_input.text()
        if port:
            self.ser = connect_uart(port)
            if self.ser and self.ser.is_open:
                self.set_connection_status(active=True)
                self.read_timer.start(200)  # Intervalo ajustado para 200 ms
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

    def handle_direction_change(self, button):
        if button == self.clockwise_button:
            command = "dir:1"
            self.terminal.append("Motor direction set to clockwise.")
        elif button == self.counterclockwise_button:
            command = "dir:0"
            self.terminal.append("Motor direction set to counter-clockwise.")
        self.terminal.ensureCursorVisible()
        self.send_uart_command(command)

    def calculate_and_send_pi(self):
        up_input = self.up_input.text()
        ts_input = self.ts_input.text()
        if up_input and ts_input:
            try:
                up = float(up_input)
                up = up * 0.01
                ts = float(ts_input)
                ki, kp = get_pi(up, ts)
                ki = ki * 1e8
                kp = kp * 1e8
                command_ki = f"ki:{ki}"
                command_kp = f"kp:{kp}"
                self.send_uart_command(command_ki)
                self.send_uart_command(command_kp)
                self.terminal.append(f"ki calculado e enviado: {ki}")
                self.terminal.append(f"kp calculado e enviado: {kp}")
                self.terminal.ensureCursorVisible()
            except ValueError as ve:
                self.terminal.append(f"Erro nos valores de entrada: {ve}")
                self.terminal.ensureCursorVisible()
            except Exception as e:
                self.terminal.append(f"Erro ao calcular PI: {e}")
                self.terminal.ensureCursorVisible()
        else:
            self.terminal.append("Por favor, insira valores para up e ts.")
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

                                # Ajuste Dinâmico do Eixo Y
                                if self.data_second:
                                    current_max = max(self.data_second)
                                    padding = current_max * 0.1  # 10% de margem
                                    # Definir um teto mínimo para evitar que o gráfico fique muito pequeno
                                    new_max = max(current_max + padding, 1)
                                    self.plot_widget_second.setYRange(0, new_max)
                                else:
                                    self.plot_widget_second.setYRange(0, 1)

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
