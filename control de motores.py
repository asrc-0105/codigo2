# Importar las bibliotecas necesarias para manejar hardware y comunicación
import gpiozero  # Biblioteca para controlar los pines GPIO en Raspberry Pi
import smbus2 as smbus  # Biblioteca para la comunicación I2C
import serial  # Biblioteca para la comunicación UART
import time  # Biblioteca para manejar tiempos y delays
from gpiozero import PWMOutputDevice, DistanceSensor  # Para el control de servomotores y sensores de distancia

# Configuración de pines y comunicación

# Configuración del pin GPIO para un LED
LED_PIN = 18  # Pin GPIO conectado al LED (ajustar según la conexión real)

# Configuración del bus I2C
I2C_BUS = 1  # Bus I2C en Raspberry Pi (1 es el bus predeterminado para modelos recientes)
I2C_ADDRESS = 0x40  # Dirección del dispositivo I2C (ajustar según el dispositivo específico)

# Configuración del puerto UART
UART_PORT = '/dev/ttyS0'  # Puerto UART en Raspberry Pi (ajustar según el dispositivo específico)
BAUD_RATE = 9600  # Tasa de baudios para la comunicación UART

# Configuración del pin GPIO para el motor DC
MOTOR_DC_PIN_A = 17  # Pin GPIO para dirección A del motor DC (ajustar según la conexión real)
MOTOR_DC_PIN_B = 27  # Pin GPIO para dirección B del motor DC (ajustar según la conexión real)
MOTOR_DC_PWM_PIN = 22  # Pin GPIO para el control de velocidad PWM del motor DC (ajustar según la conexión real)

# Configuración del pin GPIO para el servomotor
SERVO_PIN = 23  # Pin GPIO conectado al servomotor (ajustar según la conexión real)

# Configuración del pin GPIO para el sensor de distancia
SENSOR_TRIGGER_PIN = 24  # Pin GPIO para el trigger del sensor de distancia
SENSOR_ECHO_PIN = 25  # Pin GPIO para el echo del sensor de distancia

# Configuración del pin GPIO para el mecanismo de corte
CUTTER_PIN = 26  # Pin GPIO para el mecanismo de corte (ajustar según la conexión real)

# Inicialización de componentes

def initialize_gpio():
    """
    Configura los pines GPIO para su uso con el robot.
    """
    global led
    led = gpiozero.LED(LED_PIN)  # Inicializa el LED en el pin GPIO especificado

def initialize_motor_dc():
    """
    Configura los pines GPIO para el motor DC.
    """
    global motor_dc_a, motor_dc_b, motor_dc_pwm
    motor_dc_a = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_A)  # Inicializa el pin A del motor DC
    motor_dc_b = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_B)  # Inicializa el pin B del motor DC
    motor_dc_pwm = PWMOutputDevice(MOTOR_DC_PWM_PIN)  # Inicializa el control PWM del motor DC

def initialize_servo():
    """
    Configura el pin GPIO para el servomotor.
    """
    global servo
    servo = PWMOutputDevice(SERVO_PIN)  # Inicializa el servomotor en el pin GPIO especificado

def initialize_sensor():
    """
    Configura el sensor de distancia.
    """
    global sensor
    sensor = DistanceSensor(trigger_pin=SENSOR_TRIGGER_PIN, echo_pin=SENSOR_ECHO_PIN)  # Inicializa el sensor de distancia

def initialize_cutter():
    """
    Configura el mecanismo de corte.
    """
    global cutter
    cutter = gpiozero.DigitalOutputDevice(CUTTER_PIN)  # Inicializa el pin para el mecanismo de corte

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

def control_motor_dc(direction, speed):
    """
    Controla el motor DC.

    :param direction: Dirección del motor ('forward', 'backward' o 'stop')
    :param speed: Velocidad del motor (0 a 1)
    """
    try:
        if direction == 'forward':
            motor_dc_a.on()
            motor_dc_b.off()
        elif direction == 'backward':
            motor_dc_a.off()
            motor_dc_b.on()
        elif direction == 'stop':
            motor_dc_a.off()
            motor_dc_b.off()
        
        motor_dc_pwm.value = speed
    except Exception as e:
        print(f"Error al controlar el motor DC: {e}")

def control_servo(angle):
    """
    Controla el servomotor para ajustar el ángulo.

    :param angle: Ángulo a ajustar (0 a 180 grados)
    """
    try:
        # Convertir el ángulo a un valor de PWM (entre 0 y 1)
        duty_cycle = angle / 180  # Ajustar según el rango de tu servomotor
        servo.value = duty_cycle
    except Exception as e:
        print(f"Error al controlar el servomotor: {e}")

def control_cutter(state):
    """
    Controla el mecanismo de corte.

    :param state: Booleano para activar (True) o desactivar (False) el mecanismo de corte
    """
    try:
        if state:
            cutter.on()  # Activar el mecanismo de corte
        else:
            cutter.off()  # Desactivar el mecanismo de corte
    except Exception as e:
        print(f"Error al controlar el mecanismo de corte: {e}")

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
    Envía un comando a un actuador para ejecutar una acción específica.
    
    :param command: Comando para el actuador
    """
    try:
        if command == 'activate':
            # Lógica para activar el actuador
            pass
        elif command == 'deactivate':
            # Lógica para desactivar el actuador
            pass
    except Exception as e:
        print(f"Error al controlar el actuador: {e}")

def avoid_obstacle():
    """
    Detecta y evita obstáculos utilizando el sensor de distancia.
    """
    try:
        distance = sensor.distance * 100  # Convertir la distancia a centímetros
        if distance < 20:  # Si el obstáculo está a menos de 20 cm
            print("Obstáculo detectado. Deteniendo el motor...")
            control_motor_dc('stop', 0)  # Detener el motor DC
            time.sleep(1)  # Esperar 1 segundo
            # Añadir lógica para evitar el obstáculo (por ejemplo, retroceder y girar)
            print("Evitando el obstáculo...")
            control_motor_dc('backward', 0.5)  # Retroceder a velocidad media
            time.sleep(2)  # Esperar 2 segundos
            control_motor_dc('stop', 0)  # Detener el motor
    except Exception as e:
        print(f"Error al evitar obstáculos: {e}")

def run_diagnostics():
    """
    Ejecuta pruebas y diagnósticos en los componentes del robot.
    """
    try:
        print("Ejecutando pruebas de diagnóstico...")
        # Prueba de LED
        control_led(True)
        time.sleep(1)
        control_led(False)
        # Prueba de motores
        control_motor_dc('forward', 0.5)
        time.sleep(2)
        control_motor_dc('stop', 0)
        # Prueba de servomotor
        control_servo(90)
        time.sleep(2)
        control_servo(0)
        # Prueba de cortador
        control_cutter(True)
        time.sleep(2)
        control_cutter(False)
    except Exception as e:
        print(f"Error en las pruebas de diagnóstico: {e}")

def main():
    """
    Función principal que configura y maneja la comunicación entre componentes del robot.
    """
    global i2c, uart  # Declarar variables globales

    try:
        # Inicialización de componentes
        initialize_gpio()  # Configura el GPIO
        initialize_motor_dc()  # Configura el motor DC
        initialize_servo()  # Configura el servomotor
        initialize_sensor()  # Configura el sensor de distancia
        initialize_cutter()  # Configura el mecanismo de corte
        i2c = initialize_i2c()  # Configura el bus I2C
        uart = initialize_uart()  # Configura el puerto UART

        # Ejecutar diagnóstico
        run_diagnostics()

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

        # Controlar el motor DC
        print("Moviendo el motor DC hacia adelante...")
        control_motor_dc('forward', 1)  # Mover el motor DC hacia adelante a velocidad máxima
        time.sleep(5)  # Esperar 5 segundos
        print("Deteniendo el motor DC...")
        control_motor_dc('stop', 0)  # Detener el motor DC

        # Controlar el servomotor
        print("Ajustando el servomotor a 90 grados...")
        control_servo(90)  # Ajustar el servomotor a 90 grados
        time.sleep(2)  # Esperar 2 segundos
        print("Ajustando el servomotor a 0 grados...")
        control_servo(0)  # Ajustar el servomotor a 0 grados

        # Controlar el mecanismo de corte
        print("Activando el mecanismo de corte...")
        control_cutter(True)  # Activar el mecanismo de corte
        time.sleep(2)  # Esperar 2 segundos
        print("Desactivando el mecanismo de corte...")
        control_cutter(False)  # Desactivar el mecanismo de corte

        # Evitar obstáculos
        avoid_obstacle()

    except Exception as e:
        print(f"Error en la comunicación entre componentes: {e}")

    finally:
        # Limpiar la configuración de GPIO para evitar conflictos
        print("Limpiando la configuración de GPIO...")
        # gpiozero no requiere limpieza manual como en RPi.GPIO

if __name__ == "__main__":
    main()  # Ejecutar la función principal
