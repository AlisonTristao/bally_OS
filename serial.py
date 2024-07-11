import serial
import csv
import time

# Configurações da porta serial
serial_port = '/dev/ttyUSB0'  # Substitua pelo nome da porta serial correta
baud_rate = 921600  # Ajuste conforme necessário

# Inicializa a conexão serial
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Arquivo CSV onde os dados serão salvos
csv_file = 'dados_serial.csv'

# Inicializa o dicionário para armazenar os dados
dados = {'E': None, 'D': None, 'S': None}

try:
    # Abre o arquivo CSV no modo de escrita
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=';')
        # Escreve o cabeçalho no CSV
        writer.writerow(['E', 'D', 'S'])
        
        while True:
            # Lê uma linha da porta serial
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Separa o identificador e o valor
                if ':' in line:
                    identificador, valor = line.split(':')
                    if identificador in dados:
                        # Atualiza o valor no dicionário
                        dados[identificador] = valor
                    
                    # Verifica se todos os valores foram recebidos
                    if all(value is not None for value in dados.values()):
                        # Escreve os dados no arquivo CSV
                        writer.writerow([dados['E'], dados['D'], dados['S']])
                        #print(f"Escrevendo linha: E={dados['E']}, D={dados['D']}, S={dados['S']}")
                        
                        # Reseta os valores no dicionário
                        dados = {key: None for key in dados}
                        
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Leitura interrompida pelo usuário")

finally:
    # Fecha a conexão serial
    ser.close()
    print("Conexão serial fechada")
