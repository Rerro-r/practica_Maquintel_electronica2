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
unsigned long previousMillis = 0;  // Tiempo entre paquetes
unsigned long lastOledUpdate = 0;  // Control de actualización OLED
//unsigned long lastPrintMillis = 0;  // Control de impresión en Serial Monitor
uint8_t receivedData[7] = {0};  // Buffer para datos del paquete
uint8_t batteryLevel = 0;
uint8_t batteryLevelAnt = 0;
long leftEncoderTicks = 0;
long leftEncoderTicksAnt = 0;
int indexQuestion = 0;
float encoderRatio = 0.0;
int encoderType = 0;
char runCommandInit[4] = {0};
char runCommand[5] = {0};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver Optimized");
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setFrequency(902000000); // Configura a 902 MHz

  LoRa.receive();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  
  display.clearDisplay();
  display.display();
  Serial.println("Esperando datos por Serial...");
  String run_odometro_radio = ""; // Variable para almacenar el dato recibido
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
      String data = run_odometro_radio.substring(0, comaIndex1);
      data.toCharArray(runCommandInit, sizeof(runCommandInit));
    }
    int comaIndex2 = run_odometro_radio.indexOf(',', comaIndex1 + 1);
    if (comaIndex2 != -1) {
      encoderType = run_odometro_radio.substring(comaIndex1 + 1, comaIndex2).toInt();
      encoderRatio = run_odometro_radio.substring(comaIndex2 + 1).toFloat();
    }
  // Mostrar los resultados en el display
  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("serial: ");
  display.println(run_odometro_radio);
  delay(1000); // Mostrar los resultados durante 5 segundos
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

// Manejo de datos del LoRa
void handleLoRaData(int packetSize) {
  memset(receivedData, 0, sizeof(receivedData));
  if (packetSize <= sizeof(receivedData)) { 
    LoRa.readBytes(receivedData, sizeof(receivedData));}

  int indexQuestion = receivedData[0];  // Determinar tipo de pregunta

  if (indexQuestion == 1) {
    // Enviar configuración del encoder en binario por LoRa
    int bufferSize = 1 + sizeof(runCommandInit) + sizeof(encoderType) + sizeof(encoderRatio);
    uint8_t buffer[bufferSize]; // Buffer de tamaño variable
    // Construir el paquete en el buffer
    buffer[0] = 1; // Confirmación de recepción (como uint8_t)
    memcpy(buffer + 1, &runCommandInit, sizeof(runCommandInit)); // Copia el string
    buffer[1 + sizeof(runCommandInit)] = encoderType; // Copia el tipo de encoder
    memcpy(buffer + 1 + sizeof(runCommandInit) + sizeof(encoderType), &encoderRatio, sizeof(encoderRatio)); // Copia el ratio del encoder
    LoRa.beginPacket();
    // Enviar el buffer completo
    LoRa.write(buffer, bufferSize);
    LoRa.endPacket();

  //Se decide hacer checksum del index y los ticks únicamente. Si el batteryLevel se envía mal no es problema 
  } else if (indexQuestion == 2) {
    int offset = 1;
    memcpy(&leftEncoderTicks, &receivedData[offset], sizeof(leftEncoderTicks));
    offset += sizeof(leftEncoderTicks);
    // Leer batteryLevel (1 byte - tipo `uint8_t`)
    batteryLevel = (uint8_t)receivedData[offset];
    offset += sizeof(uint8_t);
    uint8_t checkSumVerificated = xorChecksum(receivedData, sizeof(receivedData));
    if (checkSumVerificated == receivedData[offset]){
      leftEncoderTicksAnt = leftEncoderTicks;
      batteryLevelAnt = batteryLevel;
    } else {
      leftEncoderTicks = leftEncoderTicksAnt;
      batteryLevel = batteryLevelAnt;
    }
    Serial.println(String(leftEncoderTicks) + "," + String(batteryLevel));
  }
}
// Manejo de datos del Serial
void handleSerialData() {
  String data = Serial.readString();
  if (data.length() > 0) {
    data.toCharArray(runCommand, sizeof(runCommand));
    // Mostrar el dato enviado y el estado en el display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(runCommand);
    display.display();
  }
}

uint8_t xorChecksum(uint8_t* buffer, int len){
  uint8_t sum = 0;
  for (int i = 0; i < 5; i++) {
      sum ^= buffer[i];
  }
  return sum;
}

