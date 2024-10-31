import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QTextEdit
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
import serial

def connect_uart(port):
    try:
        ser = serial.Serial(port, 19200, timeout=1)
        print(f"Conectado ao dispositivo UART na porta {port}.")
        return ser
    except serial.SerialException as e:
        print(f"Erro ao conectar ao UART: {e}")
        return None

class MotorControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Interface")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()

        
        self.port = '/dev/rfcomm0'
        self.ser = connect_uart(self.port)

        
        self.start_button = QPushButton("Ligar Motor")
        self.start_button.clicked.connect(self.start_motor)
        layout.addWidget(self.start_button)

        
        self.stop_button = QPushButton("Desligar Motor")
        self.stop_button.clicked.connect(self.stop_motor)
        layout.addWidget(self.stop_button)

        
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle("Velocidade do Motor")
        self.plot_widget.setLabel("left", "Velocidade", units="RPM")
        self.plot_widget.setLabel("bottom", "Tempo", units="s")
        layout.addWidget(self.plot_widget)

        
        self.data = np.zeros(100)
        self.curve = self.plot_widget.plot(self.data)

        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graph)
        self.timer.start(50)

        
        self.motor_running = False
        self.speed = 0

        
        self.connection_status = QLabel("Conexão Inativa")
        self.connection_status.setAlignment(Qt.AlignCenter)
        self.connection_status.setStyleSheet("QLabel { background-color: red; color: white; padding: 5px; }")
        layout.addWidget(self.connection_status)

        
        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        layout.addWidget(self.terminal)

        
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self.check_uart_connection)
        self.connection_timer.start(1000)  

        
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_uart_data)
        self.read_timer.start(100)  

        self.setLayout(layout)

    def start_motor(self):
        self.motor_running = True
        self.speed = 100

    def stop_motor(self):
        self.motor_running = False
        self.speed = 0

    def update_graph(self):
        
        if self.motor_running:
            self.speed += np.random.normal(0, 2)
            self.speed = max(0, self.speed)

        
        self.data = np.roll(self.data, -1)
        self.data[-1] = self.speed
        self.curve.setData(self.data)

    def check_uart_connection(self):
        
        if self.ser and self.ser.is_open:
            self.connection_status.setText("Conexão Ativa")
            self.connection_status.setStyleSheet("QLabel { background-color: green; color: white; padding: 5px; }")
        else:
            self.connection_status.setText("Conexão Inativa")
            self.connection_status.setStyleSheet("QLabel { background-color: red; color: white; padding: 5px; }")
            
            self.ser = connect_uart(self.port)

    def read_uart_data(self):
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    self.terminal.append(line)  
                    self.terminal.ensureCursorVisible()  
                    print(f"Dados recebidos: {line}")
                    
                    
            except serial.SerialException as e:
                print(f"Erro de leitura UART: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorControlApp()
    window.show()
    sys.exit(app.exec_())
