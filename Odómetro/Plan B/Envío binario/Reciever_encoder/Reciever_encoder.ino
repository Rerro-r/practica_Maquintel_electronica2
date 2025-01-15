#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

//########################### OLED ##############################
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//###############################################################

#define SCK     5
#define MISO    19
#define MOSI    27
#define SS      18
#define RST     23
#define DI0     26
#define BAND    868E6

#define PACKET_ID_POS 0
#define PACKET_LENGTH_POS 1
#define PACKET_DATA_POS 2
#define PACKET_CRC_POS 6 // Posición del CRC (después de ID, Longitud, Datos)

#define PACKET_ID_SIZE 1
#define PACKET_LENGTH_SIZE 1
#define PACKET_DATA_SIZE (sizeof(long) + sizeof(int)) // 4 + 2 = 6 bytes
#define PACKET_CRC_SIZE 2

unsigned long previousMillis = 0;  // Tiempo entre paquetes
unsigned long lastOledUpdate = 0;  // Control de actualización OLED
//unsigned long lastPrintMillis = 0;  // Control de impresión en Serial Monitor

char receivedData[10] = {0};  // Buffer para datos del paquete
int batteryLevel = 0;
long leftEncoderTicks = 0;
float distanciaRecorrida = 0.0;
int indexQuestion = 0;
float encoderRatio = 0.0;
int encoderType = 0;
float beginReset = 0.0;
String runCommand = "";

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver Optimized");

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  LoRa.setSpreadingFactor(10); // SF 10
  LoRa.setSignalBandwidth(125E3); // BW 125 kHz
  LoRa.setCodingRate4(5); // CR 4/5 (Valor por defecto, puedes probar con 6,7 u 8 para mayor robustez)
  //LoRa.setSignalBandwidth(500E3);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Configuración de LoRa para ajustar el rendimiento
  //LoRa.setSpreadingFactor(7);  // Comúnmente 7-12, prueba con 7 para mayor velocidad
  //LoRa.setSignalBandwidth(125E3);  // Ancho de banda de 125 kHz
  //LoRa.setCodingRate4(5);  // Tasa de codificación 4/5

  LoRa.receive();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  
  display.clearDisplay();
  display.display();

  Serial.println("Esperando datos por Serial...");
  String run_odometro_radio = ""; // Variable para almacenar el dato recibido

 /* // Mostrar mensaje de espera
  display.setCursor(0, 23);
  display.print("esperando datos...");
  display.display(); // Mostrar mensaje en pantalla
  */

  // Esperar hasta que llegue un dato por serial
  while (run_odometro_radio.length() == 0) {
      // Mostrar mensaje de espera
    display.setCursor(0, 23);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print("esperando datos...");
    display.display(); // Mostrar mensaje en pantalla
    if (Serial.available() > 0) {
      run_odometro_radio = Serial.readString(); // Leer el dato como cadena

      display.clearDisplay(); // Limpiar pantalla solo cuando se reciben datos
      display.setCursor(0, 16);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.print("serial recibido");
      display.display(); // Mostrar en pantalla

      delay(1000); // Pequeño retardo para evitar sobrecarga
    }
  }

  // Procesar los datos recibidos
  int comaIndex1 = run_odometro_radio.indexOf(',');
  if (comaIndex1 != -1) {
    runCommand = run_odometro_radio.substring(0, comaIndex1);

    int comaIndex2 = run_odometro_radio.indexOf(',', comaIndex1 + 1);
    if (comaIndex2 != -1) {
      encoderType = run_odometro_radio.substring(comaIndex1 + 1, comaIndex2).toInt();
      encoderRatio = run_odometro_radio.substring(comaIndex2 + 1).toFloat();
    }
  }



  // Mostrar los resultados en el display
  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("serial: ");
  display.println(run_odometro_radio);
  display.setCursor(0, 23);
  display.print("en/run: ");
  display.print(encoderType);
  display.print(",");
  display.println(runCommand);

  display.display();
  delay(5000); // Mostrar los resultados durante 5 segundos
}


void loop() {
  unsigned long currentMillis = millis();  // Obtener tiempo actual

  // Revisar datos del puerto LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleLoRaData(packetSize);
  }

  // Revisar datos del puerto Serial
  if (Serial.available() > 0) {
    handleSerialData();
  }
}
  // Actualizar la pantalla OLED cada segundo
 // if (currentMillis - lastOledUpdate >= 1000) {
  //  updateOLED();
    //lastOledUpdate = currentMillis;
  //}

  // Actualizar mensajes periódicos, si aplica
 // if (currentMillis - lastPrintMillis >= 1000) {
    //lastPrintMillis = currentMillis;
  //}
//}

// Manejo de datos del LoRa
void handleLoRaData(int packetSize) {
  memset(receivedData, 0, sizeof(receivedData));
  if (packetSize <= sizeof(receivedData)) { // Si el paquete tiene 10 bytes o menos
    int i = 0;
    while (LoRa.available()) {
      LoRa.readBytes(receivedData, sizeof(receivedData));
    }
   // receivedData[i] = '\0'; // Terminar la cadena
  } else { // Si el paquete tiene más de 7 bytes
    // Descartar los primeros bytes
    for (int i = 0; i < packetSize - sizeof(receivedData); i++) {
      LoRa.read(); // Leer y descartar
    }

    // Leer los últimos 7 bytes
    LoRa.readBytes(receivedData, sizeof(receivedData));
  //  receivedData[sizeof(receivedData)] = '\0'; // Asegurar terminación nula
  }

  int indexQuestion = receivedData[0];  // Determinar tipo de pregunta

  if (indexQuestion == 1) {
    // Enviar configuración del encoder en binario por LoRa
    int bufferSize = 1 + runCommand.length() + 1 + sizeof(encoderRatio);
    uint8_t buffer[bufferSize]; // Buffer de tamaño variable

    // Construir el paquete en el buffer
    buffer[0] = 1; // Confirmación de recepción (como uint8_t)
    memcpy(buffer + 1, runCommand.c_str(), runCommand.length()); // Copia el string
    buffer[1 + runCommand.length()] = (uint8_t)encoderType; // Copia el tipo de encoder
    memcpy(buffer + 2 + runCommand.length(), &encoderRatio, sizeof(encoderRatio)); // Copia el ratio del encoder

    LoRa.beginPacket();

    // Enviar el buffer completo
    LoRa.write(buffer, bufferSize);

    LoRa.endPacket();
  } else if (indexQuestion == 2) {
    // Procesar datos binarios del paquete recibido
    uint8_t receivedLength = receivedData[PACKET_LENGTH_POS];
            if (receivedLength == PACKET_DATA_SIZE + PACKET_CRC_SIZE) {
                long receivedLeftEncoderTicks;
                int receivedBatteryLevel;
                uint16_t receivedCRC;

                memcpy(&receivedLeftEncoderTicks, receivedData + PACKET_DATA_POS, sizeof(receivedLeftEncoderTicks));
                memcpy(&receivedBatteryLevel, receivedData + PACKET_DATA_POS + sizeof(receivedLeftEncoderTicks), sizeof(receivedBatteryLevel));
                memcpy(&receivedCRC, receivedData + PACKET_CRC_POS, sizeof(receivedCRC));
            
                uint8_t dataForCRC[PACKET_DATA_SIZE];
                memcpy(dataForCRC, receivedData + PACKET_DATA_POS, PACKET_DATA_SIZE); // Copia los datos para el CRC
                uint16_t calculatedCRC = calculateCRC(dataForCRC, PACKET_DATA_SIZE);


             //   memcpy((long*)(receivedData + PACKET_DATA_POS), sizeof(receivedLeftEncoderTicks));
                if (calculatedCRC == receivedCRC) {
                    leftEncoderTicks = receivedLeftEncoderTicks;
                    batteryLevel = receivedBatteryLevel;
                    Serial.println(String(leftEncoderTicks) + "," + String(batteryLevel)); // Imprime en formato CSV
                    //updateOLED(); // Actualizar el display
                } else {
                    Serial.println("Error de CRC");
                    Serial.print("CRC Recibido: ");
                    Serial.println(receivedCRC, HEX);
                    Serial.print("CRC Calculado: ");
                    Serial.println(calculatedCRC, HEX);
                }
            }   

  } else if (indexQuestion == 3){
     // Responder al comando recibido
    int bufferSize = 1 + runCommand.length(); // 1 byte de confirmación + longitud del string
    uint8_t buffer[bufferSize]; // Buffer de tamaño variable

    // Construir el paquete en el buffer
    buffer[0] = 3; // Confirmación de recepción (como uint8_t)
    memcpy(buffer + 1, runCommand.c_str(), runCommand.length()); // Copia el string

    LoRa.beginPacket();

    // Enviar el buffer completo
    LoRa.write(buffer, bufferSize);

    LoRa.endPacket();
  }
}

// Manejo de datos del Serial
void handleSerialData() {
  String data = Serial.readString();

  if (data.length() > 0) {
    runCommand = data;
    // Mostrar el dato enviado y el estado en el display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.println(runCommand);

    display.display();
  }
}


/*
// Actualización del OLED
void updateOLED() {
  char buffer[20]; // Buffer para almacenar la cadena formateada (tamaño suficiente para un long)
  char buffer2[15];
// Formatear la cadena
  snprintf(buffer, sizeof(buffer), "T: %d", leftEncoderTicks); // %ld para long
 // Buffer para la cadena formateada (ajusta el tamaño si es necesario)
  snprintf(buffer2, sizeof(buffer), "Bat: %d%%", batteryLevel); // %% para imprimir un % literal

  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (batteryLevel < 20) {
    display.println("Bat baja!");
  } else {
    display.println(buffer2);
  }

  display.setCursor(0, 16);
  display.println(buffer);

//  display.setCursor(0, 23);
 // display.print("Encoder: ");
 // display.print(encoderType);
 // display.print(",");
 // display.println(encoderRatio);

  display.display();
}
*/

// Función para calcular CRC-16-CCITT
uint16_t calculateCRC(uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}