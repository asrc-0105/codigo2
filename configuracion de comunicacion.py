import socket
import paho.mqtt.client as mqtt
import bleak
import can
from smbus2 import SMBus
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend
import asyncio

#  Comunicación TCP/IP 

# Configuración del servidor TCP
TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

def start_tcp_server():
    """Inicia un servidor TCP que acepta conexiones entrantes."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((TCP_IP, TCP_PORT))
    server_socket.listen(5)
    print(f"Servidor TCP iniciado en {TCP_IP}:{TCP_PORT}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Conexión aceptada desde {addr}")
        data = client_socket.recv(BUFFER_SIZE)
        print(f"Datos recibidos: {data.decode()}")
        client_socket.sendall(b"Datos recibidos")
        client_socket.close()

# Comunicación MQTT

# Configuración del cliente MQTT
MQTT_BROKER = 'mqtt.eclipse.org'
MQTT_PORT = 1883
MQTT_TOPIC = 'test/topic'

def on_message(client, userdata, message):
    """Callback que se llama cuando se recibe un mensaje MQTT."""
    print(f"Mensaje recibido en el tópico {message.topic}: {message.payload.decode()}")

def start_mqtt_client():
    """Inicia el cliente MQTT, se suscribe a un tópico y publica un mensaje."""
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(MQTT_TOPIC)
    client.loop_start()
    client.publish(MQTT_TOPIC, "Hola desde MQTT")

# Comunicación Bluetooth con Bleak 

async def scan_for_devices():
    """Escanea y muestra los dispositivos Bluetooth cercanos."""
    devices = await bleak.BleakScanner.discover()
    for device in devices:
        print(f"Dispositivo encontrado: {device.name} - {device.address}")

async def connect_to_device(address):
    """Conecta a un dispositivo Bluetooth y lee características."""
    async with bleak.BleakClient(address) as client:
        print(f"Conectado: {client.address}")
        # Lee una característica (ejemplo)
        data = await client.read_gatt_char('00002a37-0000-1000-8000-00805f9b34fb')
        print(f"Datos recibidos: {data}")

# Comunicación CAN Bus 

# Configuración de CAN Bus
CAN_INTERFACE = 'can0'
CAN_BAUDRATE = 500000

def start_can_bus():
    """Inicia la comunicación CAN Bus y envía/recibe mensajes."""
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=CAN_BAUDRATE)
    message = can.Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03, 0x04], is_extended_id=False)
    bus.send(message)
    print("Mensaje CAN enviado.")
    received_message = bus.recv()
    print(f"Mensaje CAN recibido: {received_message}")

# ----------------- Comunicación I2C -----------------

# Configuración de I2C
I2C_BUS_NUMBER = 1
I2C_ADDRESS = 0x40

def start_i2c():
    """Inicia la comunicación I2C, envía y recibe datos."""
    with SMBus(I2C_BUS_NUMBER) as bus:
        bus.write_byte(I2C_ADDRESS, 0x01)
        data = bus.read_byte(I2C_ADDRESS)
        print(f"Datos recibidos por I2C: {data}")

# ----------------- Cifrado de Datos -----------------

# Configuración del cifrado
KEY = b'Sixteen byte key'
IV = b'Sixteen byte IV '

def encrypt_data(data):
    """Cifra los datos usando AES en modo CBC."""
    cipher = Cipher(algorithms.AES(KEY), modes.CBC(IV), backend=default_backend())
    encryptor = cipher.encryptor()
    padder = padding.PKCS7(algorithms.AES.block_size).padder()
    padded_data = padder.update(data) + padder.finalize()
    encrypted_data = encryptor.update(padded_data) + encryptor.finalize()
    return encrypted_data

def decrypt_data(encrypted_data):
    """Descifra los datos usando AES en modo CBC."""
    cipher = Cipher(algorithms.AES(KEY), modes.CBC(IV), backend=default_backend())
    decryptor = cipher.decryptor()
    padded_data = decryptor.update(encrypted_data) + decryptor.finalize()
    unpadder = padding.PKCS7(algorithms.AES.block_size).unpadder()
    data = unpadder.update(padded_data) + unpadder.finalize()
    return data

# ----------------- Ejecución del Código -----------------

if __name__ == "__main__":
    # Ejecutar el servidor TCP en un hilo separado
    import threading
    tcp_thread = threading.Thread(target=start_tcp_server, daemon=True)
    tcp_thread.start()

    # Ejecutar el cliente MQTT
    start_mqtt_client()

    # Ejecutar el escaneo y conexión Bluetooth
    asyncio.run(scan_for_devices())
    # Usa la dirección de un dispositivo encontrado para la siguiente línea
    # asyncio.run(connect_to_device('device_address'))

    # Iniciar CAN Bus e I2C
    start_can_bus()
    start_i2c()
