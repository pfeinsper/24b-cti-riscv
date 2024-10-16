import serial
import time

def connect_hc06(port):
    # Conecta ao dispositivo Bluetooth HC-06 através de uma porta serial
    try:
        ser = serial.Serial(port, 9600, timeout=1)  # 9600 baud é o padrão do HC-06
        print(f"Conectado ao dispositivo HC-06 na porta {port}.")
        return ser
    except serial.SerialException as e:
        print(f"Erro ao conectar ao HC-06: {e}")
        return None

def send_command(ser, value):
    try:
        # Converte o valor para string e envia como bytes
        ser.write(value.to_bytes(1, byteorder='little', signed=True))  
        time.sleep(1)  # Espera 1 segundo
    except serial.SerialException as e:
        print(f"Erro ao enviar comando: {e}")

def main():
    # Porta serial onde o HC-06 está conectado (ex: /dev/rfcomm0 no Linux)
    port = "/dev/rfcomm0"
    
    # Tenta conectar ao HC-06
    ser = connect_hc06(port)
    if ser is None:
        return
    
    try:
        while True:
            user_input = input("Digite o valor de N (positivo ou negativo) ou 'q' para sair: ")
            
            if user_input.lower() == 'q':
                print("Encerrando o programa...")
                break
            
            try:
                # Converte a entrada para um número inteiro (pode ser positivo ou negativo)
                N = int(user_input)
                
                # Envia o valor de N
                send_command(ser, N)
                print(f"Comando enviado: {N}")
            
            except ValueError:
                print("Por favor, insira um valor numérico válido.")
    
    finally:
        ser.close()  # Fecha a conexão com o Bluetooth
        print("Conexão serial encerrada.")

if __name__ == "__main__":
    main()
