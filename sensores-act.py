# Importación de bibliotecas necesarias
import time  # Para manejar tiempos de espera y pausas
import gpiozero  # Para controlar los pines GPIO de la Raspberry Pi
from Adafruit_ADS1x15 import ADS1115  # Para leer datos analógicos desde un ADC (convertidor analógico a digital)
import board  # Para definir los pines de la Raspberry Pi
import busio  # Para manejar la comunicación I2C

# Definición de pines para los actuadores
MOTOR_PIN = 18  # Pin GPIO al que está conectado el motor
SERVO_PIN = 17  # Pin GPIO al que está conectado el servo

# Configuración del ADC (convertidor analógico a digital)
i2c = busio.I2C(board.SCL, board.SDA)  # Inicializa la comunicación I2C con los pines SCL y SDA
adc = ADS1115(i2c)  # Crea una instancia del ADC ADS1115 para leer señales analógicas

# Inicialización de componentes GPIO
motor = gpiozero.PWMOutputDevice(MOTOR_PIN, frequency=1000)  # Configura PWM para el motor con una frecuencia de 1 kHz
servo = gpiozero.PWMOutputDevice(SERVO_PIN, frequency=50)  # Configura PWM para el servo con una frecuencia de 50 Hz

# Canal del ADC para el sensor
SENSOR_CHANNEL = 0  # Canal del ADC al que está conectado el sensor (canal 0)

def read_sensor(channel=SENSOR_CHANNEL):
    """
    Lee el valor del sensor conectado al ADC y convierte el valor crudo en voltios.
    
    :param channel: Canal del ADC al que está conectado el sensor.
    :return: Valor del sensor en voltios.
    """
    try:
        # Lee el valor crudo desde el canal del ADC
        raw_value = adc.read(channel)  # Usa `read` en lugar de `read_adc`
        # Convierte el valor crudo a voltios usando el factor de conversión
        voltage = raw_value * 0.000125  # Ajusta el factor según la resolución del ADC y la referencia de voltaje
        return voltage
    except Exception as e:
        print(f"Error al leer el sensor: {e}")
        return 0

def control_motor(speed):
    """
    Controla la velocidad del motor usando PWM.
    
    :param speed: Velocidad del motor en porcentaje (0-100%).
    """
    try:
        # Limita el valor de velocidad al rango permitido (0-100%)
        speed = max(0, min(speed, 100))
        # Ajusta el ciclo de trabajo del PWM para controlar la velocidad del motor
        motor.value = speed / 100.0  # `gpiozero` usa un valor entre 0.0 y 1.0 para PWM
    except Exception as e:
        print(f"Error al controlar el motor: {e}")

def control_servo(angle):
    """
    Controla el ángulo del servo usando PWM.
    
    :param angle: Ángulo del servo en grados (0-180).
    """
    try:
        # Limita el ángulo al rango permitido (0-180 grados)
        angle = max(0, min(angle, 180))
        # Convierte el ángulo a un ciclo de trabajo de PWM (2ms - 1ms mapeado a 0-180 grados)
        duty_cycle = (angle / 180.0) * 0.1 + 0.05  # Ajusta el ciclo de trabajo del PWM para el servo
        servo.value = duty_cycle
    except Exception as e:
        print(f"Error al controlar el servo: {e}")

def main():
    """
    Función principal que lee los sensores y controla los actuadores en un bucle continuo.
    """
    try:
        while True:
            # Lee el valor del sensor y lo muestra en la consola
            sensor_value = read_sensor()
            print(f"Valor del sensor: {sensor_value:.2f} V")  # Muestra el valor del sensor en voltios
            
            # Ejemplo de control del motor y servo basado en el valor del sensor
            if sensor_value > 2.0:  # Condición arbitraria para activar el motor y ajustar el servo
                control_motor(100)  # Establece el motor a máxima velocidad
                control_servo(90)  # Ajusta el servo a 90 grados
            else:
                control_motor(0)  # Apaga el motor
                control_servo(0)  # Ajusta el servo a 0 grados
            
            # Pausa de 1 segundo antes de la siguiente lectura
            time.sleep(1)

    except KeyboardInterrupt:
        # Mensaje cuando el programa se interrumpe con Ctrl+C
        print("Deteniendo el robot...")
    finally:
        # Detiene el PWM para motor y servo
        motor.off()  # Detiene el PWM del motor
        servo.off()  # Detiene el PWM del servo

# Ejecuta la función principal cuando el script se ejecuta
if __name__ == "__main__":
    main()
