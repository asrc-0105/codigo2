import random
import time

class Robot:
    def __init__(self):
        # Inicialización de sensores y actuadores
        self.sensores = {'temperatura': 0, 'corriente': 0}
        self.estado = 'inactivo'
        self.max_intentos = 5  # Número máximo de intentos en caso de error
        self.intento = 0

    def leer_sensores(self):
        """Lee los valores de los sensores."""
        try:
            self.sensores['temperatura'] = self.obtener_temperatura()
            self.sensores['corriente'] = self.obtener_corriente()
        except Exception as e:
            print(f"Error al leer sensores: {e}")
            self.manejar_error()

    def obtener_temperatura(self):
        """Simulación de lectura de temperatura."""
        return random.randint(20, 40)  # Simulación de lectura

    def obtener_corriente(self):
        """Simulación de lectura de corriente."""
        return random.randint(0, 15)  # Simulación de lectura

    def tomar_decision(self):
        """Toma decisiones basadas en las lecturas de sensores."""
        self.leer_sensores()
        
        temperatura = self.sensores['temperatura']
        corriente = self.sensores['corriente']
        
        if temperatura > 35:
            self.estado = 'sobrecalentado'
            self.accion_sobrecalentado()
        elif corriente > 12:
            self.estado = 'sobrecarga'
            self.accion_sobrecarga()
        else:
            self.estado = 'operativo'
            self.accion_operativo()

    def accion_sobrecalentado(self):
        """Acciones específicas para estado sobrecalentado."""
        print("¡Alerta! El robot está sobrecalentado.")
        self.enfriar_robot()
    
    def accion_sobrecarga(self):
        """Acciones específicas para estado de sobrecarga."""
        print("¡Alerta! El robot está en sobrecarga.")
        self.reducir_carga()
    
    def accion_operativo(self):
        """Acciones cuando el robot está en estado operativo."""
        print("El robot está en funcionamiento normal.")
    
    def enfriar_robot(self):
        """Simulación de enfriamiento del robot."""
        print("Enfriando el robot...")
        time.sleep(2)  # Simulación de tiempo de enfriamiento
    
    def reducir_carga(self):
        """Simulación de reducción de carga."""
        print("Reduciendo carga...")
        time.sleep(2)  # Simulación de tiempo de reducción
    
    def manejar_error(self):
        """Maneja errores en la lectura de sensores."""
        self.intento += 1
        if self.intento >= self.max_intentos:
            print("Error crítico. Deteniendo el robot.")
            self.estado = 'error'
            return
        print("Reintentando...")
        time.sleep(1)  # Esperar antes de reintentar
        self.leer_sensores()  # Reintentar lectura de sensores

    def ejecutar(self):
        """Ejecución continua del robot con manejo de errores."""
        while self.estado != 'error':
            self.tomar_decision()
            time.sleep(1)  # Espera entre ciclos de decisión

if __name__ == "__main__":
    robot = Robot()
    robot.ejecutar()
