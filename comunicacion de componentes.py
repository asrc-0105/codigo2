# Importar las bibliotecas necesarias para manejar hardware y comunicación
import gpiozero  # Biblioteca para controlar los pines GPIO en Raspberry Pi
import smbus2 as smbus  # Biblioteca para la comunicación I2C
import serial  # Biblioteca para la comunicación UART
import time  # Biblioteca para manejar tiempos y delays

# Configuración de pines y comunicación

# Configuración del pin GPIO para un LED
LED_PIN = 18  # Pin GPIO conectado al LED (ajustar según la conexión real)

# Configuración del bus I2C
I2C_BUS = 1  # Bus I2C en Raspberry Pi (1 es el bus predeterminado para modelos recientes)
I2C_ADDRESS = 0x40  # Dirección del dispositivo I2C (ajustar según el dispositivo específico)

# Configuración del puerto UART
UART_PORT = '/dev/ttyS0'  # Puerto UART en Raspberry Pi (ajustar según el dispositivo específico)
BAUD_RATE = 9600  # Tasa de baudios para la comunicación UART

# Inicialización de componentes

def initialize_gpio():
    """
    Configura los pines GPIO para su uso con el robot.
    """
    global led
    led = gpiozero.LED(LED_PIN)  # Inicializa el LED en el pin GPIO especificado

def initialize_i2c():
    """
    Configura y retorna una instancia del bus I2C para la comunicación.
    
    :return: Instancia del bus I2C
    """
    try:
        i2c = smbus.SMBus(I2C_BUS)  # Crear instancia del bus I2C
        return i2c
    except Exception as e:
        print(f"Error al inicializar I2C: {e}")
        raise

def initialize_uart():
    """
    Configura y retorna una instancia del puerto UART para la comunicación.
    
    :return: Instancia del puerto UART
    """
    try:
        uart = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)  # Crear instancia del puerto UART con timeout
        return uart
    except Exception as e:
        print(f"Error al inicializar UART: {e}")
        raise

# Funciones de control de hardware

def control_led(state):
    """
    Controla el estado de un LED conectado a un pin GPIO.

    :param state: Booleano para encender (True) o apagar (False) el LED
    """
    try:
        if state:
            led.on()  # Encender el LED
        else:
            led.off()  # Apagar el LED
    except Exception as e:
        print(f"Error al controlar el LED: {e}")

def read_i2c_data(register):
    """
    Lee un byte de datos desde un registro específico de un dispositivo I2C.

    :param register: Registro desde el cual leer el dato
    :return: Valor leído del registro o None si ocurre un error
    """
    try:
        return i2c.read_byte_data(I2C_ADDRESS, register)  # Leer un byte del registro especificado
    except Exception as e:
        print(f"Error al leer datos I2C del registro {register}: {e}")
        return None

def write_i2c_data(register, value):
    """
    Escribe un byte de datos en un registro específico de un dispositivo I2C.

    :param register: Registro en el cual escribir el dato
    :param value: Valor a escribir en el registro
    """
    try:
        i2c.write_byte_data(I2C_ADDRESS, register, value)  # Escribir un byte en el registro especificado
    except Exception as e:
        print(f"Error al escribir datos I2C en el registro {register}: {e}")

def read_uart_data():
    """
    Lee datos desde el puerto UART.

    :return: Datos leídos del puerto UART o None si no hay datos disponibles
    """
    try:
        if uart.in_waiting > 0:  # Verificar si hay datos disponibles en el buffer de entrada
            return uart.read(uart.in_waiting).decode('utf-8')  # Leer y decodificar datos del buffer
        return None
    except Exception as e:
        print(f"Error al leer datos UART: {e}")
        return None

def write_uart_data(data):
    """
    Escribe datos en el puerto UART.

    :param data: Datos a enviar al puerto UART
    """
    try:
        uart.write(data.encode('utf-8'))  # Codificar y enviar datos al puerto UART
    except Exception as e:
        print(f"Error al escribir datos UART: {e}")

def control_actuator(command):
    """
    Envía un comando a un actuador a través del puerto UART.

    :param command: Comando a enviar al actuador
    """
    write_uart_data(command)  # Enviar el comando al actuador a través del puerto UART

# Función principal

def main():
    """
    Función principal que configura y maneja la comunicación entre componentes del robot.
    """
    global i2c, uart  # Declarar variables globales

    try:
        # Inicialización de componentes
        initialize_gpio()  # Configura el GPIO
        i2c = initialize_i2c()  # Configura el bus I2C
        uart = initialize_uart()  # Configura el puerto UART

        # Encender y apagar el LED como prueba
        print("Encendiendo el LED...")
        control_led(True)  # Encender el LED
        time.sleep(1)  # Esperar 1 segundo
        print("Apagando el LED...")
        control_led(False)  # Apagar el LED

        # Leer y escribir datos en I2C
        print("Leyendo datos del dispositivo I2C...")
        data = read_i2c_data(0x00)  # Leer datos del registro 0x00
        if data is not None:
            print(f"Datos I2C leídos: {data}")
        print("Escribiendo datos en el dispositivo I2C...")
        write_i2c_data(0x01, 0xFF)  # Escribir 0xFF en el registro 0x01

        # Leer y escribir datos en UART
        print("Leyendo datos del puerto UART...")
        uart_data = read_uart_data()  # Leer datos del puerto UART
        if uart_data:
            print(f"Datos UART leídos: {uart_data}")
        print("Escribiendo datos en el puerto UART...")
        write_uart_data("Comando: ON")  # Enviar comando al puerto UART

        # Controlar el actuador
        print("Controlando el actuador...")
        control_actuator("MOVE")  # Enviar comando para controlar el actuador

    except Exception as e:
        print(f"Error en la comunicación entre componentes: {e}")

    finally:
        # Limpiar la configuración de GPIO para evitar conflictos
        print("Limpiando la configuración de GPIO...")
        # No es necesario hacer `gpiozero.cleanup()` como en RPi.GPIO. 
        # gpiozero no requiere limpieza manual al contrario de RPi.GPIO

if __name__ == "__main__":
    main()  # Ejecutar la función principal
