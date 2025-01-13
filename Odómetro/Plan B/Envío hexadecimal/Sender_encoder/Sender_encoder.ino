#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <LoRa.h>

//########################### LORA ##############################
#define SCK     5    // GPIO5  -- SCK
#define MISO    19   // GPIO19 -- MISO
#define MOSI    27   // GPIO27 -- MOSI
#define SS      18   // GPIO18 -- CS
#define RST     23   // GPIO14 -- RESET (If Lora does not work, replace it with GPIO14)
#define DI0     26   // GPIO26 -- IRQ(Interrupt Request)
#define BAND    868E6

String rssi = "RSSI --";
String packSize = "--";
String packet ;
volatile bool transmissionFinished = true; // Variable volátil para la interrupción
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
#define c_LeftEncoderPinB 12  
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
long _LeftEncoderTicks = 0;
String Dato = "";
//#########################################################

//########################## Voltaje ###############################
// Definir el pin ADC que se utilizará
const int batteryPin = 39; // Utiliza un pin ADC como GPIO
// Constantes para el cálculo del voltaje
const float adcResolution = 4095.0;
const float referenceVoltage = 3.3;
const float voltageDividerRatio = 0.41; // Ajustar según tu divisor de voltaje
// Constantes para el mapeo del porcentaje de la batería
const float minBatteryVoltage = 3.3; // Voltaje mínimo de la batería considerada vacía
const float maxBatteryVoltage = 8.0; // Voltaje máximo de la batería considerada llena
const int numReadings = 10; 
//#########################################################

//########################## Variables #############################
unsigned long lastDisplayUpdate = 0;  // Tiempo del último update de pantalla
unsigned long lastBatteryUpdate = 0;  // Tiempo del último cálculo de batería
unsigned long lastCommandUpdate = 0;
unsigned long lastCommandAsk = 0;
unsigned long lastLoRaPacketSent = 0;
int batteryLevel = 0;  // Almacenará el nivel de batería
unsigned long lastLoRaSend = 0;  // Tiempo del último paquete LoRa enviado
unsigned long timeBetweenPackets = 0;  // Tiempo entre el último y el actual envío de paquete

char receivedData[256];
char receivedDataSetup[256] = {0};
int encoderType = 0;
float beginReset = 0.0;
float distanciaRecorrida = 0.0;
bool reset = false;
bool stopSending = false;
int ticksNecesarios = 0;
float encoderRatio = 0.0; // medido en metros
float distancia = 0.0;
String runCommand = "";
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
  if (!LoRa.begin(868E6)) {
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
  delay(7000);
  // Una vez LoRa esté listo, enviar el paquete


int packetSize = 0;

// Enviar el paquete indefinidamente hasta recibir una respuesta
  while (packetSize == 0) {
    LoRa.beginPacket();
    uint8_t request = 1; // Solicitud en formato binario (1: pedir configuración del encoder)
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

  uint8_t receivedBytes[256];
  int receivedBytesCount = 0;
  if (packetSize <= sizeof(receivedBytes)) {
      receivedBytesCount = LoRa.readBytes(receivedBytes, packetSize);
    }
    else {
      for (int i = 0; i < packetSize; i++) {
         LoRa.read();
      }
    }

// Leer el paquete recibido en un buffer binario
  String hexString = bytesToHexString(receivedBytes, receivedBytesCount);
  Serial.print("Hex recibido en setup: ");
  Serial.println(hexString);
  uint8_t dataBytes[128];
  int dataLength = hexStringToBytes(hexString, dataBytes);
  if(dataLength > 0){
    int offset = 0;
    char runCommand[32];
    int encoderType = 0;
    float encoderRatio = 0.0;

    memcpy(runCommand, &dataBytes[offset], min((int)sizeof(runCommand) - 1, dataLength - offset)); //Protección contra desbordamiento
    runCommand[min((int)sizeof(runCommand) - 1, dataLength - offset)] = '\0';
    offset += strlen(runCommand) + 1;
    if (offset + sizeof(encoderType) <= dataLength) {
      memcpy(&encoderType, &dataBytes[offset], sizeof(encoderType));
      offset += sizeof(encoderType);
    }
    if (offset + sizeof(encoderRatio) <= dataLength) {
      memcpy(&encoderRatio, &dataBytes[offset], sizeof(encoderRatio));
      offset += sizeof(encoderRatio);
    }
    Serial.println("Comando recibido: " + String(runCommand));
    Serial.println("Tipo de encoder recibido: " + String(encoderType));
    Serial.println("Ratio del encoder recibido: " + String(encoderRatio));
  }

  //##########################################################
  //#########################################################

  //########################## ENCODER ###############################
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP); // sets pin A as input with pull-up
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP); // sets pin B as input with pull-up
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterruptA, RISING);
  //#########################################################
  LoRa.receive();
}

void loop() {
  unsigned long currentMillis = millis();
  // Actualizar la pantalla OLED cada segundo
  // Actualizar la pantalla OLED cada 2.5 segundos
  if (currentMillis - lastDisplayUpdate >= 1000) {
    updateOLED();
    lastDisplayUpdate = currentMillis;
  }

  // Actualizar el nivel de batería cada 60 segundos
  if (currentMillis - lastBatteryUpdate >= 10000) {
    batteryLevel = getBatteryLevel();
    lastBatteryUpdate = currentMillis;
  }

//  if (stopSending == false) {
      // Enviar datos por LoRa
  if (currentMillis - lastLoRaSend >= 30) {
    //if (currentMillis - lastLoRaSend != 61) {
     // Serial.println(currentMillis - lastLoRaSend);
   //// }
    sendLoRaPacket();
    lastLoRaSend = currentMillis;
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
  //lastDisplayUpdate = millis();
  if (batteryLevel > 10){
    char displayBuffer[50]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bat: %d\nT: %ld", batteryLevel, _LeftEncoderTicks);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 8); // Ajusta la posición vertical si es necesario
    display.print(displayBuffer); // Imprime la cadena formateada
    display.display();
  } else {
    char displayBuffer[50]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bateria baja! %d\nT: %ld", batteryLevel, _LeftEncoderTicks);

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

// Actualiza el nivel de batería
//void updateBatteryLevel() {
  //lastBatteryUpdate = millis();
  //batteryLevel = getBatteryLevel();
//}

// Verifica si se puede enviar un paquete LoRa
//bool canSendLoRaPacket(unsigned long currentMillis) {
 // if (currentMillis - lastLoRaSend >= 7) {
   // Serial.println(currentMillis - lastLoRaSend);
 //   return LoRa.beginPacket();
 // } else {
   // return false;
  //}



// Envía datos por LoRa
void sendLoRaPacket() {
  uint8_t buffer[7];
  buffer[0] = 2;
  memcpy(buffer + 1, &_LeftEncoderTicks, sizeof(_LeftEncoderTicks));
  memcpy(buffer + 5, &batteryLevel, sizeof(batteryLevel));

  String hexToSend = bytesToHexString(buffer, sizeof(buffer));
  Serial.print("Enviando Hex: ");
  Serial.println(hexToSend);

  int numBytes = hexToSend.length() / 2;
  uint8_t bytesToSend[numBytes];
  hexStringToBytes(hexToSend, bytesToSend);

  LoRa.beginPacket();
  LoRa.write(bytesToSend, numBytes);
  LoRa.endPacket();

}
void onTxDone() {
  transmissionFinished = true;
}
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
  int percentage = map(voltage * 100, 330, 800, 0, 100);
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

String bytesToHexString(const uint8_t* bytes, int len) {
  String hexString = "";
  for (int i = 0; i < len; i++) {
    char hexByte[3];
    sprintf(hexByte, "%02X", bytes[i]);
    hexString += hexByte;
  }
  return hexString;
}

int hexStringToBytes(const String& hexString, uint8_t* bytes) {
  int len = hexString.length();
  if (len % 2 != 0) {
    return -1;
  }
  for (int i = 0; i < len; i += 2) {
    String byteStr = hexString.substring(i, i + 2);
    char byteChars[3];
    byteStr.toCharArray(byteChars, 3);
    bytes[i / 2] = strtol(byteChars, NULL, 16);
  }
  return len / 2;
}
