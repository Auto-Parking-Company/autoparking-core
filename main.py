# Import tesseract, open cv
import cv2
import pytesseract

#!!import RPi.GPIO as GPIO

import re
import time

import paho.mqtt.client as mqtt

#? 
#?
#? _________________________________________________________  INICIALIZAÇÃO _________________________________________________________


# Read sensor HC-SR04
SENSOR_TRIGGER = 23
SENSOR_ECHO = 24

# Gates
GATE_ENTRY = 17
GATE_EXIT = 27

# Set GPIO mode
#!!GPIO.setmode(GPIO.BCM)
#!!GPIO.setup(SENSOR_TRIGGER, GPIO.OUT)
#!!GPIO.setup(SENSOR_ECHO, GPIO.IN)

# Set gates
#!!GPIO.setup(GATE_ENTRY, GPIO.OUT)
#!!GPIO.setup(GATE_EXIT, GPIO.OUT)


# Broker address
broker_address = "broker.hivemq.com"
broker_port = 1883
broker_keepalive = 60   

# Configura o caminho para o executável do Tesseract OCR
pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# Variável para armazenar a lista de placas permitidas
list_of_plates_allowed = []



#? 
#?
#? _________________________________________________________  MÉTODOS _________________________________________________________

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("parking-auto-sihs-si/allowed")


def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    plate_allowed_string = str(msg.payload)
    plate_allowed_string = plate_allowed_string[2:-1]
    print(plate_allowed_string)
    list_of_plates_allowed.append(plate_allowed_string)

def on_disconnect(client, userdata, rc):
    print("Disconnected with result code "+str(rc))

def read_distance():
   #!! GPIO.output(SENSOR_TRIGGER, True)
    time.sleep(0.00001)
    #!!GPIO.output(SENSOR_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    #!!while GPIO.input(SENSOR_ECHO) == 0:
    #!!    StartTime = time.time()
    #!!while GPIO.input(SENSOR_ECHO) == 1:
    #!!    StopTime = time.time()
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    return distance


def open_gate(gate_pin, time_open):
    print ("Opening gate", gate_pin)
    #!!GPIO.output(gate_pin, True)
    #!!time.sleep(time_open)
    #!!GPIO.output(gate_pin, False)

# Função para aplicar filtro de pré-processamento na imagem
def pre_processamento(imagem):
    # Converte a imagem para escala de cinza
    imagem_cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
    
    # Aplica filtro Gaussiano para reduzir o ruído
    imagem_filtrada = cv2.GaussianBlur(imagem_cinza, (5, 5), 0)
    
    # Aplica limiarização para binarizar a imagem
    _, imagem_binaria = cv2.threshold(imagem_filtrada, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Melhora 

    return imagem_binaria



#? 
#?
#? _________________________________________________________  LOOP START _________________________________________________________

# MQTT Callbacks
park_mqtt_client = mqtt.Client()

park_mqtt_client.on_connect = on_connect
park_mqtt_client.on_message = on_message
park_mqtt_client.on_disconnect = on_disconnect  

park_mqtt_client.connect(broker_address, broker_port, broker_keepalive)
park_mqtt_client.loop_start()



# Carrega a imagem da placa do carro
cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    # Aplica o pré-processamento na imagem
    imagem_processada = pre_processamento(frame)

    # Extrai o texto da imagem
    texto = pytesseract.image_to_string(imagem_processada, lang='eng', config="--psm 11 ")

    # Exibe a imagem e o texto extraído
    cv2.imshow("Imagem", frame)
    cv2.imshow("Imagem processada", imagem_processada)

    # Placa do carro (AAA)
    placa = re.findall("[A-Z0-9]{3}", texto)
    
    # Exibe a placa do carro
    print("Placa: ", placa)

    time.sleep(0.01)
    
    # Faz a leitura do sensor de proximidade
    sensor_output = read_distance()
    #!print(sensor_output)

    #? 
    #? 
    #? _________________________________________________________ SISTEMA DE AUTOMAÇÃO _________________________________________________________
    # Case 1: Carro se aproxima do sensor na entrada
    if sensor_output < 10 and sensor_output > 0:
        # Abre a cancela de entrada
        open_gate(GATE_ENTRY, 3)


    # Case 2: Carro se aproxima do sensor na saída
    for p in placa:
        if p in list_of_plates_allowed:
                # Abre a cancela de saída
                open_gate(GATE_EXIT, 3)
                list_of_plates_allowed.remove(p)
                print("\n\n\nPlaca removida da lista de permitidas: ", p)
                break
    
        
    
        
 
     
    # Printa a lista de placas permitidas
    print("Lista de placas permitidas: ", list_of_plates_allowed)

    # Fecha a janela se a tecla ESC for pressionada
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()
park_mqtt_client.loop_stop()