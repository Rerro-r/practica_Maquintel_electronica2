#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <LoRa.h>

// Definiciones LoRa
#define SCK    5
#define MISO   19
#define MOSI   27
#define SS     18
#define RST    23
#define DI0    26
#define BAND   868E6

// Definiciones OLED
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Definiciones Encoder
#define c_LeftEncoderPinA 34
#define c_LeftEncoderPinB 12
#ifdef LeftEncoderIsReversed
#define ENCODER_DIRECTION -1
#else
#define ENCODER_DIRECTION 1
#endif
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0;

// Definiciones Batería
const int batteryPin = 39;
const float adcResolution = 4095.0;
const float referenceVoltage = 3.3;
const float voltageDividerRatio = 0.41;

// Variables
unsigned long lastDisplayUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastLoRaSend = 0;
int batteryLevel = 0;

// Función para convertir datos a string de forma eficiente
void dataToString(char *buffer, int battery, long encoder) {
  snprintf(buffer, 20, "%d %08ld", battery, encoder); // Formato más compacto
}

void setup() {
  Serial.begin(115200);

  // Inicializar OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();

  // Inicializar LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(7); // Ajustar el spreading factor para mayor alcance o velocidad
  LoRa.setSignalBandwidth(125E3); // Ajustar el ancho de banda

  // Inicializar Encoder
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP);
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterruptA, RISING);
}

void loop() {
  unsigned long currentMillis = millis();

  // Actualizar OLED cada segundo
  if (currentMillis - lastDisplayUpdate >= 1000) {
    updateOLED();
    lastDisplayUpdate = currentMillis;
  }

  // Actualizar batería cada 2 minutos
  if (currentMillis - lastBatteryUpdate >= 120000) {
    batteryLevel = getBatteryLevel();
    lastBatteryUpdate = currentMillis;
  }

  // Enviar datos por LoRa cada 30 ms (ajustar según necesidad)
  if (currentMillis - lastLoRaSend >= 30) {
    sendLoRaPacket();
    lastLoRaSend = currentMillis;
  }
}

void updateOLED() {
  char displayBuffer[20];
  snprintf(displayBuffer, sizeof(displayBuffer), "Bat:%d T:%ld", batteryLevel, _LeftEncoderTicks);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.print(displayBuffer);
  display.display();
}

void sendLoRaPacket() {
  char dataBuffer[20];
  dataToString(dataBuffer, batteryLevel, _LeftEncoderTicks);

  LoRa.beginPacket();
  LoRa.write('2'); // ID del paquete
  LoRa.print(dataBuffer); // Enviar los datos como un string
  LoRa.endPacket();
}

int getBatteryLevel() {
  float voltage = (analogRead(batteryPin) / adcResolution) * referenceVoltage / voltageDividerRatio;
  int percentage = map(voltage * 100, 330, 800, 0, 100);
  return (percentage <= 10) ? -1 : constrain(percentage, 0, 100);
}

void HandleLeftMotorInterruptA() {
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);
  _LeftEncoderTicks += ENCODER_DIRECTION * (_LeftEncoderBSet ? -1 : 1);
}