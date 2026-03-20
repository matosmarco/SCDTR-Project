import serial
import time
import csv

# CONFIGURAÇÃO DA PORTA
SERIAL_PORT = '/dev/ttyACM0'  # Ajusta para a tua porta (ex: 'COM3' ou '/dev/ttyACM0')
BAUD_RATE = 115200
#FILE_NAME = "kp_low.csv"
FILE_NAME =  "kp_medium.csv"
#FILE_NAME =  "kp_high.csv"
#FILE_NAME =  "kp_with_ki_low.csv"
#FILE_NAME =  "b02.csv"

def run_automated_test():
    try:
        # Abrir ligação Serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Aguarda o Pico estabilizar
        
        # 1. ENVIAR COMANDOS DE CONFIGURAÇÃO INICIAL
        print("A enviar configurações iniciais...")
        commands = ["f 0 1", "ff 0 0", "a 0 0", "sb 0"]
        for cmd in commands:
            ser.write(f"{cmd}\n".encode())
            print(f"-> {cmd}")
            time.sleep(0.1) # Pequena pausa entre comandos

        # 2. DEFINIR A SEQUÊNCIA DE TESTE
        # (Referência, Duração em segundos)
        test_sequence = [
            ("r 0 0", 5),
            ("r 0 30", 10),
            ("r 0 0", 8),
            ("r 0 15", 10)
        ]

        print(f"\nA iniciar captura para {FILE_NAME}...")
        
        with open(FILE_NAME, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["id", "lux", "duty", "ref", "time_ms"]) # Cabeçalho

            for cmd_ref, duration in test_sequence:
                print(f"A executar: {cmd_ref} durante {duration}s")
                ser.write(f"{cmd_ref}\n".encode())
                
                start_step = time.time()
                while (time.time() - start_step) < duration:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("s b"):
                        parts = line.split()
                        if len(parts) >= 7:
                            # Formato: s b id y u r t -> extrair de partes[2] a partes[6]
                            writer.writerow(parts[2:7])

        # 3. FINALIZAÇÃO
        ser.write(b"S b 0\n") # Para o stream
        ser.write(b"r 0 0\n") # Apaga a luz
        ser.close()
        print(f"\nTeste concluído com sucesso! Ficheiro guardado: {FILE_NAME}")

    except Exception as e:
        print(f"Erro durante a execução: {e}")

if __name__ == "__main__":
    run_automated_test()
