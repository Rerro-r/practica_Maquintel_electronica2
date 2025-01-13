#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

// Configuración OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Configuración LoRa
#define SCK    5
#define MISO   19
#define MOSI   27
#define SS     18
#define RST    23
#define DI0    26
#define BAND   868E6

// Variables
char receivedData[13] = {0};
int batteryLevel = 0;
long leftEncoderTicks = 0;
String runCommand = "";
int encoderType = 0;
float encoderRatio = 0.0;
unsigned long lastOledUpdate = 0; // Tiempo de la última actualización del OLED

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver Optimized");

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.receive();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Esperando datos Serial...");
  display.display();

  // Procesar datos iniciales del Serial (con manejo de errores)
  if (Serial.available() > 0) {
    String serialData = Serial.readStringUntil('\n');
    serialData.trim();

    int commaIndex1 = serialData.indexOf(',');
    if (commaIndex1 != -1) {
      runCommand = serialData.substring(0, commaIndex1);
      int commaIndex2 = serialData.indexOf(',', commaIndex1 + 1);
      if (commaIndex2 != -1) {
        String encoderTypeStr = serialData.substring(commaIndex1 + 1, commaIndex2);
        String encoderRatioStr = serialData.substring(commaIndex2 + 1);
        encoderType = encoderTypeStr.toInt();
        encoderRatio = encoderRatioStr.toFloat();

        // Mostrar datos recibidos del serial
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Comando: ");
        display.println(runCommand);
        display.print("Tipo Encoder: ");
        display.println(encoderType);
        display.print("Ratio Encoder: ");
        display.println(encoderRatio);
        display.display();
        delay(2000);
      } else {
        Serial.println("Error: Formato de datos serial incorrecto (falta segunda coma).");
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Error serial");
          display.display();
          delay(2000);
      }
    } else {
      Serial.println("Error: Formato de datos serial incorrecto (falta primera coma).");
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Error serial");
        display.display();
        delay(2000);
    }
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    if (packetSize > 0 && packetSize <= sizeof(receivedData)) {
      LoRa.readBytes(receivedData, packetSize);
      receivedData[packetSize] = '\0';
      handleLoRaData(packetSize);
    } else if (packetSize > sizeof(receivedData)) {
      for (int i = 0; i < packetSize; i++) {
        LoRa.read();
      }
      Serial.println("Paquete LoRa demasiado grande, descartado");
    } else {
      Serial.println("Paquete LoRa vacío o inválido");
    }
  }

  // Actualizar el OLED cada segundo
  if (millis() - lastOledUpdate >= 1000) {
    updateOLED();
    lastOledUpdate = millis();
  }
}

void handleLoRaData(int packetSize) {
  int indexQuestion = receivedData[0];
  if (indexQuestion == '1') {
    // Manejar la solicitud de configuración (enviar datos por LoRa)
    int bufferSize = 1 + runCommand.length() + sizeof(encoderType) + sizeof(encoderRatio);
    uint8_t buffer[bufferSize];
    buffer[0] = '1';
    memcpy(buffer + 1, runCommand.c_str(), runCommand.length());
    memcpy(buffer + 1 + runCommand.length(), &encoderType, sizeof(encoderType));
    memcpy(buffer + 1 + runCommand.length() + sizeof(encoderType), &encoderRatio, sizeof(encoderRatio));

    LoRa.beginPacket();
    LoRa.write(buffer, bufferSize);
    LoRa.endPacket();

  } else if (indexQuestion == '2') {
    char batteryStr[4] = {0};
    char encoderStr[9] = {0};
    int batteryIndex = 0;
    int encoderIndex = 0;
    bool spaceFound = false;

    for (int i = 1; i < packetSize; i++) {
      if (receivedData[i] == ' ') {
        spaceFound = true;
        continue;
      }
      if (!spaceFound) {
        batteryStr[batteryIndex++] = receivedData[i];
      } else {
        encoderStr[encoderIndex++] = receivedData[i];
      }
    }

    batteryLevel = atoi(batteryStr);
    leftEncoderTicks = atol(encoderStr);
    Serial.print("Bateria: ");
    Serial.print(batteryLevel);
    Serial.print(", Encoder: ");
    Serial.println(leftEncoderTicks);
  } else if (indexQuestion == '3') {
    // Manejar el comando (recibir y procesar un comando)
  }
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Bat: ");
  display.print(batteryLevel);
  display.print("%");
  display.setCursor(0, 10);
  display.print("Encoder: ");
  display.println(leftEncoderTicks);
  display.display();
}