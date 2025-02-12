
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"


//////////////////////// LoRa /////////////////////////////////////

#define RF_FREQUENCY                                902000000 // Hz
#define TX_OUTPUT_POWER                             17        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

////////////////////////////// OLED ///////////////////////////////////

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

#define DEMO_DURATION 3000

int demoMode = 0;
int counter = 1;

///////////////////// Variables traspasadas //////////////////////////

//########################## Encoder ###############################
#define c_LeftEncoderPinA 6
#define c_LeftEncoderPinB 7  
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
long _LeftEncoderTicks = 0;
//#########################################################

//########################## Voltaje ###############################
// Definir el pin ADC que se utilizará
const int batteryPin = 2; // Utiliza un pin ADC como GPIO
// Constantes para el cálculo del voltaje
const float adcResolution = 4095.0;
const float referenceVoltage = 3.3;
const float voltageDividerRatio = 0.41; // Ajustar según tu divisor de voltaje
// Constantes para el mapeo del porcentaje de la batería
const float minBatteryVoltage = 5.0; // Voltaje mínimo de la batería considerada vacía
const float maxBatteryVoltage = 8.0; // Voltaje máximo de la batería considerada llena
const int numReadings = 10; 
//#########################################################

//########################## Variables #############################
unsigned long lastDisplayUpdate = 0;  // Tiempo del último update de pantalla
unsigned long lastBatteryUpdate = 0;  // Tiempo del último cálculo de batería
unsigned long lastCommandUpdate = 0;
unsigned long lastCommandAsk = 0;
unsigned long lastLoRaPacketSent = 0;
unsigned long previousMillis = 0;
int batteryLevel = 0;  // Almacenará el nivel de batería
unsigned long timeBetweenPackets = 0;  // Tiempo entre el último y el actual envío de paquete
char receivedData[256] = {0};
int encoderType = 0; // 2 bytes
float encoderRatio = 0.0; // medido en metros 4 bytes
String runCommandInit = "";
bool txDone = false;
bool rxDone = false;
//#########################################################


void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);


  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
  
  if(Radio.GetStatus()){
  Serial.println("Configuración LoRa establecida");
  delay(1000);
  }

  VextON();
  display.init();
  display.setFont(ArialMT_Plain_16);
  Serial.println("OLED configurado");
  display.clear();

unsigned long startMillis = 0;
bool waitingPeriod = false;

while (!rxDone) {  // Mientras rxDone sea falso, seguimos ejecutando
  unsigned long currentMillis = millis();

  // Si no estamos en espera, ejecutamos el loop por 3 segundos
  if (!waitingPeriod) {
    if (startMillis == 0) {
      startMillis = currentMillis;
    }

    if (currentMillis - startMillis < 3000) { // Ejecutar por 3 segundos
      // Verificamos si la condición original sigue siendo válida
      if (rxDone) break;  // Salimos del while si rxDone cambia

      // Envío constante durante los 3 segundos
      uint8_t requestData[1] = {1}; 
      Radio.Send(requestData, 1);
      Radio.IrqProcess();
     // delay(100);
      //Serial.println("Encoder pedido");

    } else {
      waitingPeriod = true;  // Pasamos a la fase de espera
      startMillis = millis(); // Reiniciamos el temporizador
        // if (lora_idle) {
          //lora_idle = false;
          //Radio.Rx(0);
          //Serial.println("Modo recepción activo");
      //Radio.IrqProcess();
     //   }
    }
  } 

  // Fase de espera de 3 segundos antes de volver a ejecutar
  else if (currentMillis - startMillis < 2000) {
          if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0);

        Serial.println("modo recepción");
      }
    //waitingPeriod = false;  // Volvemos a ejecutar el loop
    //startMillis = 0;
    Serial.println("esperando respuesta");
      Radio.IrqProcess();
      //Radio.IrqProcess();
      //delay(20); // Pequeña pausa para no saturar el sistema

    // Si la condición ha cambiado, salir del while
    if (rxDone) break;
  }     else {
      waitingPeriod = false; // Volver a la fase de transmisión
      startMillis = 0;
      Serial.println("Regresando a transmisión...");
    }
}



 
  Serial.println("Paquete recibido.");

  //########################## ENCODER ###############################
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP); // sets pin A as input with pull-up
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP); // sets pin B as input with pull-up
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterruptA, FALLING);
}



void loop(){
  unsigned long currentMillis = millis();

  // Actualizar la pantalla OLED cada 1.5 segundos
  if (currentMillis - lastDisplayUpdate >= 1500) {
    updateOLED();
    lastDisplayUpdate = currentMillis;
  }
  // Actualizar el nivel de batería cada 30 segundos
  if (currentMillis - lastBatteryUpdate >= 5000) {
    batteryLevel = getBatteryLevel();
    lastBatteryUpdate = currentMillis;
  }

	if(lora_idle == true){
    if(currentMillis - lastLoRaPacketSent >= 33){
      sendLoRaPacket(); 
      lora_idle = false;
      Serial.println(currentMillis - lastLoRaPacketSent);
      lastLoRaPacketSent = currentMillis;
    }
	}
  Radio.IrqProcess();
}

void sendLoRaPacket(){
  uint8_t buffer[7] = {0}; // Tamaño total: 1 byte para el ID, 4 bytes para _LeftEncoderTicks, 1 bytes para batteryLevel y 1 byte para checksum
  uint8_t bat8 = (uint8_t)batteryLevel;
  buffer[0] = 2;
  memcpy(buffer + 1, &_LeftEncoderTicks, sizeof(_LeftEncoderTicks));
  memcpy(buffer + 5, &bat8, sizeof(bat8));
  // Leer batteryLevel (1 byte - tipo `uint8_t`)
  buffer[6] = xorChecksum(buffer, sizeof(buffer)); 
  // Enviar los bytes directamente
  Radio.Send(buffer, sizeof(buffer));
}

void OnTxDone( void )
{
	lora_idle = true;
  txDone = true;
  Serial.println("enviado");
}

void OnTxTimeout( void )
{
    Serial.println("TX Timeout......");
    lora_idle = true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Serial.println("recibí algo");
  rxDone = true;
  // Leer el paquete recibido en un buffer binario
  // 1 byte confirmación + 4 bytes command + 4 bytes int + 4 bytes float
  uint8_t bufferInit[13]; // Buffer para almacenar los datos recibidos
  int i = sizeof(bufferInit);
  // Procesar los datos recibidos
  int offsetInit = 1;
  char runCommandInit[4] = {0};      // Cadena para el comando recibido
  // Leer y procesar el comando
  memcpy(bufferInit, payload, sizeof(bufferInit));
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
  lora_idle = true;
}

void updateOLED() {
  float distancia = Distance();
    if (batteryLevel > 10){
    char displayBuffer[70]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bat: %d%%\nT: %ld\nDist: %f", batteryLevel, _LeftEncoderTicks, distancia);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, displayBuffer); // Imprime la cadena formateada
    display.display();
  } else {
    char displayBuffer[70]; // Ajusta el tamaño si necesitas más espacio
    snprintf(displayBuffer, sizeof(displayBuffer), "Bateria baja! %d%%\nT: %ld\nDist: %f", batteryLevel, _LeftEncoderTicks, distancia);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, displayBuffer); // Imprime la cadena formateada
    display.display();
  }
}

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

int getBatteryLevel() {
  float voltage = (analogRead(batteryPin) / adcResolution) * referenceVoltage / voltageDividerRatio;
  int percentage = map(voltage * 100, 500, 800, 0, 100);
  return constrain(percentage, 0, 100);
}

uint8_t xorChecksum(uint8_t* buffer, int len){
  uint8_t sum = 0;
  for (int i = 0; i < 5; i++) {
      sum ^= buffer[i];
  }
  return sum;
}

void HandleLeftMotorInterruptA() {
  //Serial.println("inteR");
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);   // leer el pin de entrada
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #endif
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
