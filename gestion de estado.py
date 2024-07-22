import time
import gpiozero
from gpiozero import PWMOutputDevice, DistanceSensor
from picamera import PiCamera
import cv2

# Configuración de pines
LED_PIN = 18
MOTOR_DC_PIN_A = 17
MOTOR_DC_PIN_B = 27
MOTOR_DC_PWM_PIN = 22
SERVO_PIN = 23
SENSOR_TRIGGER_PIN = 24
SENSOR_ECHO_PIN = 25
CUTTER_PIN = 26

# Estados del robot
IDLE = 0
MOVING = 1
CUTTING = 2
AVOIDING_OBSTACLE = 3
ERROR = 4

# Inicialización de componentes
def initialize_gpio():
    """Inicializa los componentes de hardware del robot."""
    global led, motor_dc_a, motor_dc_b, motor_dc_pwm, servo, sensor, cutter, camera
    led = gpiozero.LED(LED_PIN)
    motor_dc_a = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_A)
    motor_dc_b = gpiozero.DigitalOutputDevice(MOTOR_DC_PIN_B)
    motor_dc_pwm = PWMOutputDevice(MOTOR_DC_PWM_PIN)
    servo = PWMOutputDevice(SERVO_PIN)
    sensor = DistanceSensor(trigger_pin=SENSOR_TRIGGER_PIN, echo_pin=SENSOR_ECHO_PIN)
    cutter = gpiozero.DigitalOutputDevice(CUTTER_PIN)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 24

# Control de hardware
def control_led(state):
    """Enciende o apaga el LED según el estado."""
    led.on() if state else led.off()

def control_motor_dc(direction, speed):
    """Controla el motor DC según la dirección y velocidad proporcionadas."""
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

def control_servo(angle):
    """Ajusta el ángulo del servomotor."""
    duty_cycle = angle / 180
    servo.value = duty_cycle

def control_cutter(state):
    """Activa o desactiva el cortador."""
    cutter.on() if state else cutter.off()

# Funciones para el manejo de estados
def handle_idle():
    """Maneja el estado IDLE del robot."""
    print("Estado: IDLE - Robot inactivo")
    control_led(False)  # Apagar LED
    control_motor_dc('stop', 0)  # Detener motor

def handle_moving():
    """Maneja el estado MOVING del robot."""
    print("Estado: MOVING - Robot en movimiento")
    control_led(True)  # Encender LED
    control_motor_dc('forward', 0.5)  # Mover hacia adelante

def handle_cutting():
    """Maneja el estado CUTTING del robot."""
    print("Estado: CUTTING - Cortando cable")
    control_cutter(True)  # Activar cortador
    time.sleep(2)  # Cortar por 2 segundos
    control_cutter(False)  # Desactivar cortador

def handle_avoiding_obstacle():
    """Maneja el estado AVOIDING_OBSTACLE del robot."""
    print("Estado: AVOIDING_OBSTACLE - Evitando obstáculo")
    control_motor_dc('stop', 0)  # Detener motor
    time.sleep(1)
    control_motor_dc('backward', 0.5)  # Retroceder
    time.sleep(2)
    control_motor_dc('stop', 0)  # Detener motor

def handle_error(error_message):
    """Maneja los errores del robot."""
    print(f"Estado: ERROR - {error_message}")
    control_led(False)  # Apagar LED
    control_motor_dc('stop', 0)  # Detener motor
    # Aquí puedes añadir más acciones en caso de error

# Función principal
def main():
    """Función principal del robot que gestiona los estados y la operación general."""
    state = IDLE
    initialize_gpio()
    
    while True:
        try:
            if state == IDLE:
                handle_idle()
                # Cambiar estado basado en condiciones
                state = MOVING
            elif state == MOVING:
                handle_moving()
                # Cambiar estado basado en condiciones
                state = AVOIDING_OBSTACLE
            elif state == CUTTING:
                handle_cutting()
                # Cambiar estado basado en condiciones
                state = IDLE
            elif state == AVOIDING_OBSTACLE:
                handle_avoiding_obstacle()
                # Cambiar estado basado en condiciones
                state = MOVING
            else:
                handle_error("Estado desconocido")
            
            time.sleep(1)
        
        except Exception as e:
            handle_error(f"Excepción: {e}")

if __name__ == "__main__":
    main()
