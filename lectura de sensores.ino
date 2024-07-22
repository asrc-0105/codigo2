#include <EEPROM.h> // Incluye la librería EEPROM para almacenamiento en memoria

// Declaración de pines para los sensores
const int corrientePin = A0; // Pin analógico para el sensor de corriente
const int voltajePin = A1;  // Pin analógico para el sensor de voltaje

// Configuración de constantes para los sensores
const float voltajeReferencia = 5.0; // Voltaje de referencia del Arduino
const float corrienteSensibilidad = 0.185; // Sensibilidad del sensor ACS712 (mV/A)
const int tamanoVentana = 10; // Número de muestras para el filtro de media móvil

// Variables para almacenar las lecturas
float voltaje;
float corriente;
float corrienteOffset = 0.0; // Offset calibrado del sensor de corriente

// Arreglos y variables para el filtro de media móvil
float corrienteMuestras[tamanoVentana];
float voltajeMuestras[tamanoVentana];
int indiceMuestra = 0; // Índice para almacenar las muestras en el arreglo

// Configuración inicial del Arduino
void setup() {
  Serial.begin(9600); // Inicializa la comunicación serial a 9600 baudios
  
  // Inicializa las muestras con 0
  for (int i = 0; i < tamanoVentana; i++) {
    corrienteMuestras[i] = 0.0;
    voltajeMuestras[i] = 0.0;
  }

  // Calibración del sensor de corriente
  calibrarSensorCorriente();
}

// Función para calibrar el sensor de corriente
void calibrarSensorCorriente() {
  Serial.println("Calibrando sensor de corriente...");
  long sumaLecturas = 0;
  const int numLecturas = 100;

  // Leer varias veces el valor del sensor para encontrar el offset
  for (int i = 0; i < numLecturas; i++) {
    sumaLecturas += analogRead(corrientePin);
    delay(10); // Espera para evitar lecturas demasiado rápidas
  }
  
  // Calcular el offset promedio
  corrienteOffset = (sumaLecturas / numLecturas) * voltajeReferencia / 1024.0;
  Serial.print("Offset de corriente calibrado: ");
  Serial.println(corrienteOffset);
}

// Función para convertir lecturas analógicas a valores de corriente
float convertirCorriente(int lectura) {
  return ((lectura * voltajeReferencia) / 1024.0 - corrienteOffset) / corrienteSensibilidad;
}

// Función para convertir lecturas analógicas a valores de voltaje
float convertirVoltaje(int lectura) {
  return (lectura * voltajeReferencia) / 1024.0;
}

// Función para aplicar el filtro de media móvil
float aplicarFiltroMediaMovil(float muestra[], float nuevoValor) {
  // Eliminar el valor más antiguo y añadir el nuevo valor
  muestra[indiceMuestra] = nuevoValor;
  indiceMuestra = (indiceMuestra + 1) % tamanoVentana;
  
  // Calcular el promedio de las muestras
  float suma = 0.0;
  for (int i = 0; i < tamanoVentana; i++) {
    suma += muestra[i];
  }
  return suma / tamanoVentana;
}

// Función para leer y procesar datos de los sensores
void leerYProcesarDatos() {
  float lecturaVoltaje = convertirVoltaje(analogRead(voltajePin));
  float lecturaCorriente = convertirCorriente(analogRead(corrientePin));
  
  // Verificar si los valores de voltaje y corriente son válidos
  if (lecturaVoltaje >= 0 && lecturaCorriente >= 0) {
    // Aplicar el filtro de media móvil
    voltaje = aplicarFiltroMediaMovil(voltajeMuestras, lecturaVoltaje);
    corriente = aplicarFiltroMediaMovil(corrienteMuestras, lecturaCorriente);
    
    // Mostrar los resultados en el monitor serial
    Serial.print("Voltaje (filtrado): ");
    Serial.print(voltaje);
    Serial.print(" V, Corriente (filtrada): ");
    Serial.print(corriente);
    Serial.println(" A");
  } else {
    Serial.println("Error: Datos no válidos.");
  }
}

// Bucle principal del programa
void loop() {
  leerYProcesarDatos();
  
  // Utilizar un temporizador para reducir la frecuencia de las lecturas
  delay(500); // Espera 500 milisegundos
}
