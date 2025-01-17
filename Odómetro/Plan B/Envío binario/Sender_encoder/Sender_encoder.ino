#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <LoRa.h>
//#include <mySD.h>

//ext::File myFile;
//########################### LORA ##############################
#define SCK     5    // GPIO5  -- SCK
#define MISO    19   // GPIO19 -- MISO
#define MOSI    27   // GPIO27 -- MOSI
#define SS      18   // GPIO18 -- CS
#define RST     23  // GPIO14 -- RESET (If Lora does not work, replace it with GPIO14)
#define DI0     26   // GPIO26 -- IRQ(Interrupt Request)
#define BAND    868E6
String rssi = "RSSI --";
String packSize = "--";
String packet ;
//volatile bool transmissionFinished = true; // Variable volátil para la interrupción
//#########################################################
//########################### OLED ##############################
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//#########################################################
//########################## Encoder ###############################
#define c_LeftEncoderPinA 34
#define c_LeftEncoderPinB 35  
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
long _LeftEncoderTicks = 0;
//#########################################################
//########################## Voltaje ###############################
// Definir el pin ADC que se utilizará
const int batteryPin = 39; // Utiliza un pin ADC como GPIO
// Constantes para el cálculo del voltaje
const float adcResolution = 4095.0;
const float referenceVoltage = 3.3;
const float voltageDividerRatio = 0.41; // Ajustar según tu divisor de voltaje
// Constantes para el mapeo del porcentaje de la batería
const float minBatteryVoltage = 5.0; // Voltaje mínimo de la batería considerada vacía
const float maxBatteryVoltage = 8.0; // Voltaje máximo de la batería considerada llena
const int numReadings = 10; 
//#########################################################
/*
//################ SD #####################################
#define  SD_CLK     14
#define  SD_MISO    2
#define  SD_MOSI    15
#define  SD_CS      13

#define  Select    LOW   //  Low CS means that SPI device Selected
#define  DeSelect  HIGH  //  High CS means that SPI device Deselected

//#########################################################
*/
//########################## Variables #############################
unsigned long lastDisplayUpdate = 0;  // Tiempo del último update de pantalla
unsigned long lastBatteryUpdate = 0;  // Tiempo del último cálculo de batería
unsigned long lastCommandUpdate = 0;
unsigned long lastCommandAsk = 0;
unsigned long lastLoRaPacketSent = 0;
int batteryLevel = 0;  // Almacenará el nivel de batería
unsigned long lastLoRaSend = 0;  // Tiempo del último paquete LoRa enviado
unsigned long timeBetweenPackets = 0;  // Tiempo entre el último y el actual envío de paquete
char receivedData[256] = {0};
int encoderType = 0; // 2 bytes
bool stopSending = false;
float encoderRatio = 0.0; // medido en metros 4 bytes
String runCommandInit = "";
//#########################################################
void setup() {
  Serial.begin(115200);
  //########################## OLED ###############################
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  //########################### LORA ##############################
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Sender Test");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.onTxDone(onTxDone);
  //#########################################################
  //############# Petición y espera de datos de encoder ######
  // Esperar hasta que LoRa esté listo para transmitir
  while (!LoRa.beginPacket()) {  // Bucle hasta que LoRa esté listo
    Serial.println("Esperando a que LoRa esté listo para enviar...");
    delay(100);  // Esperar 100ms antes de intentar nuevamente
  }
  delay(3000);
  // Una vez LoRa esté listo, enviar el paquete
  int packetSize = 0;
  // Enviar el paquete indefinidamente hasta recibir una respuesta
  while (packetSize == 0) {
    LoRa.beginPacket();
    uint8_t request = 1; // Solicitud de configuración del encoder
    LoRa.write(request);
    LoRa.endPacket();
    Serial.println("Encoder pedido");
    LoRa.receive();
    // Esperar a que se reciba un paquete
    delay(500); // Retraso para evitar enviar paquetes demasiado rápido
    packetSize = LoRa.parsePacket(); // Verificar si se ha recibido un paquete
    if (packetSize == 0) {
      Serial.println("Esperando paquete...");
    }
  }
  Serial.println("Paquete recibido.");
  // Leer el paquete recibido en un buffer binario
  // 1 byte confirmación + 4 bytes command + 4 bytes int + 4 bytes float
  uint8_t bufferInit[13]; // Buffer para almacenar los datos recibidos
  int i = 0;
  while (LoRa.available() && i < sizeof(bufferInit)) {
    bufferInit[i++] = LoRa.read();
  }
  // Procesar los datos recibidos
  int offsetInit = 1;
  char runCommandInit[4] = {0};      // Cadena para el comando recibido
  // Leer y procesar el comando
  memcpy(runCommandInit, &bufferInit[offsetInit], sizeof(runCommandInit));
  offsetInit += sizeof(runCommandInit);          // Avanzar el puntero
  // Leer y procesar el tipo de encoder
  if (offsetInit + sizeof(encoderType) <= i) {
    memcpy(&encoderType, &bufferInit[offsetInit], sizeof(encoderType));
    offsetInit += sizeof(encoderType);
  }
  // Leer y procesar el ratio del encoder
  if (offsetInit + sizeof(encoderRatio) <= i) {
    memcpy(&encoderRatio, &bufferInit[offsetInit], sizeof(encoderRatio));
    offsetInit += sizeof(encoderRatio);
  }
  // Mostrar los valores procesados
  Serial.println("Comando recibido: " + String(runCommandInit));
  Serial.println("Tipo de encoder recibido: " + String(encoderType));
  Serial.println("Ratio del encoder recibido: " + String(encoderRatio));
  
  //##########################################################
  //#########################################################
  //########################## ENCODER ###############################
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP); // sets pin A as input with pull-up
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP); // sets pin B as input with pull-up
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterruptA, RISING);
  //#########################################################
  LoRa.receive();
}
/*
//######################## SD ##############################
  Serial.print("Initializing SD card...");
    pinMode(SD_CS, OUTPUT);

    if (!SD.begin(SD_CS)) {
      Serial.println("initialization failed!");
      return;
    }
    Serial.println("initialization done.");
  }

//###########################################################
*/

void loop() {
  unsigned long currentMillis = millis();
  // Actualizar la pantalla OLED cada 1.5 segundos
  if (currentMillis - lastDisplayUpdate >= 1500) {
    updateOLED();
    lastDisplayUpdate = currentMillis;
  }
  // Actualizar el nivel de batería cada 30 segundos
  if (currentMillis - lastBatteryUpdate >= 30000) {
    batteryLevel = getBatteryLevel();
    lastBatteryUpdate = currentMillis;
  }
//  if (stopSending == false) {
  // Enviar datos por LoRa
  if (currentMillis - lastLoRaSend >= 33) {
    Serial.println(currentMillis - lastLoRaSend);
    sendLoRaPacket();
    lastLoRaSend = currentMillis;
   // writeAnything();
   // readAnything();
  //  }
  }
}
 /* if (shouldUpdateCommand) {
    askCommand();
  }
  if (shouldStopSending) { // falta opción de cuando no le llega nada
    if (LoRa.parsePacket()) {
      Serial.println("Paquete LoRa detectado.");
      readCommand();
    } else {
        Serial.println("No se recibió ningún paquete.");
    }
  }
}
*/
/*
// Verifica si es momento de actualizar la pantalla
bool shouldUpdateDisplay(unsigned long currentMillis) {
  return currentMillis - lastDisplayUpdate >= 2500;
}
bool shouldUpdateCommand(unsigned long currentMillis) {
  return currentMillis - lastCommandUpdate >= 1800000;
}
bool shouldStopSending(unsigned long currentMillis) {
  Serial.print("lastCommandAsk: ");
  Serial.println(lastCommandAsk);
  Serial.print("currentMillis: ");
  Serial.println(currentMillis);
  return currentMillis - lastCommandAsk >= 1000;
}
*/
// Actualiza la pantalla OLED
void updateOLED() {
    float distancia = Distance();
    if (batteryLevel > 10){
    char displayBuffer[70]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bat: %d%%\nT: %ld\nDist: %f", batteryLevel, _LeftEncoderTicks, distancia);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 8); // Ajusta la posición vertical si es necesario
    display.print(displayBuffer); // Imprime la cadena formateada
    display.display();
  } else {
    char displayBuffer[70]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bateria baja! %d%%\nT: %ld\nDist: %f", batteryLevel, _LeftEncoderTicks, distancia);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 8); // Ajusta la posición vertical si es necesario
    display.print(displayBuffer); // Imprime la cadena formateada
    display.display();
  }
}
// Verifica si es momento de actualizar el nivel de batería
//bool shouldUpdateBattery(unsigned long currentMillis) {
  //return currentMillis - lastBatteryUpdate >= 60000;
//}
/*
void askCommand() {
  if (canSendLoRaPacket()) {
    LoRa.beginPacket();
    LoRa.print(3);
    LoRa.endPacket();
    lastCommandAsk = millis();
    stopSending = true;
    Serial.print("Stop Sending: ");
    Serial.println(stopSending);
  }
}
*/

// Envía datos por LoRa
//Se decide hacer checksum del index y los ticks únicamente. Si el batteryLevel se envía mal no es problema 
void sendLoRaPacket() {
  uint8_t buffer[7] = {0}; // Tamaño total: 1 byte para el ID, 4 bytes para _LeftEncoderTicks, 1 bytes para batteryLevel y 1 byte para checksum
  uint8_t bat8 = (uint8_t)batteryLevel;
  buffer[0] = 2;
  memcpy(buffer + 1, &_LeftEncoderTicks, sizeof(_LeftEncoderTicks));
  memcpy(buffer + 5, &bat8, sizeof(bat8));
  buffer[6] = xorChecksum(buffer, sizeof(buffer)); 
  LoRa.beginPacket();
  LoRa.write(buffer, sizeof(buffer));
  LoRa.endPacket();
}

//void onTxDone() {
  //transmissionFinished = true;
//}
/*
// Maneja la recepción de datos LoRa
void readCommand() {
  Serial.println("Comando de respaldo recibido!");
  int i = 0;
  while (LoRa.available() && i < sizeof(receivedData) - 1) {
    receivedData[i++] = LoRa.read();
  }
  receivedData[i] = '\0';  // Terminar la cadena
  // Convertir y procesar el valor recibido
  runCommand = String(receivedData);
  Serial.print("Run Command: ");
  Serial.println(runCommand);
  stopSending = false;
}
*/
int getBatteryLevel() {
  float voltage = (analogRead(batteryPin) / adcResolution) * referenceVoltage / voltageDividerRatio;
  int percentage = map(voltage * 100, 500, 800, 0, 100);
  return constrain(percentage, 0, 100);
}

void HandleLeftMotorInterruptA() {
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);   // leer el pin de entrada
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #endif
}

uint8_t xorChecksum(uint8_t* buffer, int len){
  uint8_t sum = 0;
  for (int i = 0; i < 5; i++) {
      sum ^= buffer[i];
  }
  return sum;
}

float Distance() {
  float distanciaRecorrida = 0.0;
  if (encoderType == 1) { // guía de cable
      distanciaRecorrida = round(((int(_LeftEncoderTicks) * 0.0372 * 3.1416) / 1024) * 1 * 100.0) / 100.0;
  }
  else if (encoderType == 2) { // carrete
      distanciaRecorrida = round(((int(_LeftEncoderTicks) * 0.0225 * 3.1416) / 1024) * 1.0216 * 100.0) / 100.0;
  }
  else if (encoderType == 3) { // personalizado
      distanciaRecorrida = round(((int(_LeftEncoderTicks) * encoderRatio * 3.1416) / 1024) * 1 * 100.0) / 100.0;
  } 
  return -1 * distanciaRecorrida;
}

/*
void writeAnything() {
  // Open the file for writing (overwrite existing content)
  myFile = SD.open("test.csv", F_APPEND); 
  //Si en lugar de sobrescribir el archivo quieres añadir datos al final 
  //del mismo en cada iteración del bucle, debes usar el modo F_APPEND
  // en lugar de FILE_WRITE.

  if (myFile) {
    myFile.println("any," + String(_LeftEncoderTicks));
    myFile.close();
  } else {
    Serial.println("error opening test.txt");
  }
}

void readAnything(){
  myFile = SD.open("test.csv");
  if (myFile) {
    Serial.println("Contenido de test.csv:");
    while (myFile.available()) {
      String line = myFile.readStringUntil('\n');
      line.trim(); // Elimina espacios en blanco al inicio y final
      if (line.length() > 0) { // Evita imprimir líneas vacías
        Serial.println(line);
      }
    }
    myFile.close();
  } else {
    Serial.println("Error al abrir test.csv para lectura");
  }
}
*/