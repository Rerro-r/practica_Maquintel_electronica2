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
#define PACKET_ID_SIZE 1
#define PACKET_LENGTH_SIZE 1
#define PACKET_DATA_SIZE (sizeof(long) + sizeof(int)) 

unsigned long previousMillis = 0;  // Tiempo entre paquetes
unsigned long lastOledUpdate = 0;  // Control de actualización OLED
//unsigned long lastPrintMillis = 0;  // Control de impresión en Serial Monitor

char receivedData[9] = {0};  // Buffer para datos del paquete
int batteryLevel = 0;
long leftEncoderTicks = 0;
float distanciaRecorrida = 0.0;
int indexQuestion = 0;
float encoderRatio = 0.0;
int encoderType = 0;
float beginReset = 0.0;
String runCommand = "";

// Define los bytes de inicio y fin
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE = 0x55;

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
    int bytesRead = LoRa.readBytes(receivedData, min(packetSize, (int)sizeof(receivedData)));
  

    // Verifica la longitud y los bytes de inicio y fin
    if (bytesRead >= 3 && receivedData[0] == START_BYTE && receivedData[bytesRead - 1] == END_BYTE) {
      uint8_t indexQuestion = receivedData[1];
      uint8_t receivedLength = receivedData[2];
  
  
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
      Serial.println(bufferSize);
    } else if (indexQuestion == 2 && receivedLength == PACKET_DATA_SIZE && bytesRead == 3 + receivedLength + 1) {

      long receivedLeftEncoderTicks;
      int receivedBatteryLevel;
      uint16_t receivedCRC;

      memcpy(&receivedLeftEncoderTicks, receivedData + 3, sizeof(receivedLeftEncoderTicks));
      memcpy(&receivedBatteryLevel, receivedData + 3 + sizeof(receivedLeftEncoderTicks), sizeof(receivedBatteryLevel));

      leftEncoderTicks = receivedLeftEncoderTicks;
      batteryLevel = receivedBatteryLevel;
      Serial.println(String(leftEncoderTicks) + "," + String(batteryLevel)); // Imprime en formato CSV
      //updateOLED(); // Actualizar el display
      

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
