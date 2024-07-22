import gpiozero
import smbus2 as smbus
import serial
import time
from gpiozero import PWMOutputDevice, DistanceSensor
from picamera import PiCamera  # Requiere la biblioteca de la cámara de Raspberry Pi
import numpy as np
import cv2

# Configuración de pines y comunicación
LED_PIN = 18
I2C_BUS = 1
I2C_ADDRESS = 0x40
UART_PORT = '/dev/ttyS0'
BAUD_RATE = 9600
MOTOR_DC_PIN_A = 17
MOTOR_DC_PIN_B = 27
MOTOR_DC_PWM_PIN = 22
SERVO_PIN = 23
SENSOR_TRIGGER_PIN = 24
SENSOR_ECHO_PIN = 25
CUTTER_PIN = 26
BATTERY_SENSOR_PIN = 5
CURRENT_SENSOR_PIN = 6  # Pin para el sensor de corriente
VOLTMETER_SENSOR_PIN = 7  # Pin para el sensor de voltaje

# Inicialización de componentes
def initialize_gpio():
    global led
    led = gpiozero.LED(LED_PIN)

def initialize_motor_dc():
    global motor_dc_a, motor_dc_b, motor_dc_pwm
    motor_dc_a = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_A)
    motor_dc_b = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_B)
    motor_dc_pwm = PWMOutputDevice(MOTOR_DC_PWM_PIN)

def initialize_servo():
    global servo
    servo = PWMOutputDevice(SERVO_PIN)

def initialize_sensor():
    global sensor
    sensor = DistanceSensor(trigger_pin=SENSOR_TRIGGER_PIN, echo_pin=SENSOR_ECHO_PIN)

def initialize_cutter():
    global cutter
    cutter = gpiozero.DigitalOutputDevice(CUTTER_PIN)

def initialize_battery_sensor():
    global battery_sensor
    battery_sensor = gpiozero.AnalogInputDevice(BATTERY_SENSOR_PIN)

def initialize_current_sensor():
    global current_sensor
    current_sensor = gpiozero.AnalogInputDevice(CURRENT_SENSOR_PIN)

def initialize_voltmeter_sensor():
    global voltmeter_sensor
    voltmeter_sensor = gpiozero.AnalogInputDevice(VOLTMETER_SENSOR_PIN)

def initialize_camera():
    global camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 24

def initialize_i2c():
    try:
        i2c = smbus.SMBus(I2C_BUS)
        return i2c
    except Exception as e:
        print(f"Error al inicializar I2C: {e}")
        raise

def initialize_uart():
    try:
        uart = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        return uart
    except Exception as e:
        print(f"Error al inicializar UART: {e}")
        raise

# Funciones de control de hardware
def control_led(state):
    try:
        if state:
            led.on()
        else:
            led.off()
    except Exception as e:
        print(f"Error al controlar el LED: {e}")

def control_motor_dc(direction, speed):
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
    try:
        duty_cycle = angle / 180
        servo.value = duty_cycle
    except Exception as e:
        print(f"Error al controlar el servomotor: {e}")

def control_cutter(state):
    try:
        if state:
            cutter.on()
        else:
            cutter.off()
    except Exception as e:
        print(f"Error al controlar el mecanismo de corte: {e}")

def read_battery_level():
    try:
        voltage = battery_sensor.value * 3.3
        print(f"Nivel de batería: {voltage}V")
        return voltage
    except Exception as e:
        print(f"Error al leer el nivel de batería: {e}")
        return None

def read_current_level():
    try:
        current = current_sensor.value * 3.3
        print(f"Nivel de corriente: {current}A")
        return current
    except Exception as e:
        print(f"Error al leer el nivel de corriente: {e}")
        return None

def read_voltage_level():
    try:
        voltage = voltmeter_sensor.value * 3.3
        print(f"Nivel de voltaje: {voltage}V")
        return voltage
    except Exception as e:
        print(f"Error al leer el nivel de voltaje: {e}")
        return None

def read_i2c_data(register):
    try:
        return i2c.read_byte_data(I2C_ADDRESS, register)
    except Exception as e:
        print(f"Error al leer datos I2C del registro {register}: {e}")
        return None

def write_i2c_data(register, value):
    try:
        i2c.write_byte_data(I2C_ADDRESS, register, value)
    except Exception as e:
        print(f"Error al escribir datos I2C en el registro {register}: {e}")

def read_uart_data():
    try:
        if uart.in_waiting > 0:
            return uart.read(uart.in_waiting).decode('utf-8')
        return None
    except Exception as e:
        print(f"Error al leer datos UART: {e}")
        return None

def write_uart_data(data):
    try:
        uart.write(data.encode('utf-8'))
    except Exception as e:
        print(f"Error al escribir datos UART: {e}")

def control_actuator(command):
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
    try:
        distance = sensor.distance * 100
        if distance < 20:
            print("Obstáculo detectado. Deteniendo el motor...")
            control_motor_dc('stop', 0)
            time.sleep(1)
            print("Evitando el obstáculo...")
            control_motor_dc('backward', 0.5)
            time.sleep(2)
            control_motor_dc('stop', 0)
    except Exception as e:
        print(f"Error al evitar obstáculos: {e}")

def detect_and_cut_cable():
    try:
        # Captura de imagen y procesamiento
        camera.capture('image.jpg')
        image = cv2.imread('image.jpg')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Encuentra contornos en la imagen
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            print("Cable detectado, procediendo al corte...")
            control_cutter(True)
            time.sleep(2)
            control_cutter(False)
        else:
            print("No se detectó cable.")
    except Exception as e:
        print(f"Error al detectar y cortar el cable: {e}")

def run_diagnostics():
    try:
        print("Ejecutando pruebas de diagnóstico...")
        control_led(True)
        time.sleep(1)
        control_led(False)
        control_motor_dc('forward', 0.5)
        time.sleep(2)
        control_motor_dc('stop', 0)
        control_servo(90)
        time.sleep(2)
        control_servo(0)
        control_cutter(True)
        time.sleep(2)
        control_cutter(False)
        detect_and_cut_cable()
    except Exception as e:
        print(f"Error en las pruebas de diagnóstico: {e}")

def main():
    global i2c, uart, camera
    
    try:
        initialize_gpio()
        initialize_motor_dc()
        initialize_servo()
        initialize_sensor()
        initialize_cutter()
        initialize_battery_sensor()
        initialize_current_sensor()
        initialize_voltmeter_sensor()
        initialize_camera()
        i2c = initialize_i2c()
        uart = initialize_uart()

        run_diagnostics()

        print("Leyendo datos del dispositivo I2C...")
        data = read_i2c_data(0x00)
        if data is not None:
            print(f"Datos I2C leídos: {data}")
        print("Escribiendo datos en el dispositivo I2C...")
        write_i2c_data(0x01, 0xFF)

        print("Leyendo datos del puerto UART...")
        uart_data = read_uart_data()
        if uart_data:
            print(f"Datos UART leídos: {uart_data}")
        print("Escribiendo datos en el puerto UART...")
        write_uart_data("Comando: ON")

        print("Moviendo el motor DC hacia adelante...")
        control_motor_dc('forward', 1)
        time.sleep(5)
        print("Deteniendo el motor DC...")
        control_motor_dc('stop', 0)

        print("Ajustando el servomotor a 90 grados...")
        control_servo(90)
        time.sleep(2)
        print("Ajustando el servomotor a 0 grados...")
        control_servo(0)

        print("Activando el mecanismo de corte...")
        control_cutter(True)
        time.sleep(2)
        print("Desactivando el mecanismo de corte...")
        control_cutter(False)

        avoid_obstacle()

        print("Nivel de batería...")
        read_battery_level()

        print("Nivel de corriente...")
        read_current_level()

        print("Nivel de voltaje...")
        read_voltage_level()

    except Exception as e:
        print(f"Error en la comunicación entre componentes: {e}")

    finally:
        print("Limpiando la configuración de GPIO...")

if __name__ == "__main__":
    main()
