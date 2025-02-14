
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"



#define RF_FREQUENCY                                902000000 // Hz

#define TX_OUTPUT_POWER                             22        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       6         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            50
#define BUFFER_SIZE                                 13 // Define the payload size here


static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

volatile bool lora_idle = true;
volatile States_t state;
volatile bool sleepMode = false;
volatile int16_t Rssi,rxSize;

/////////////////// OLED /////////////////////////////////////////

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

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


////////////////// Variables traspasadas ////////////////////////

unsigned long lastDisplayUpdate = 0;  // Tiempo del último update de pantalla
unsigned long lastBatteryUpdate = 0;  // Tiempo del último cálculo de batería
unsigned long lastCommandUpdate = 0;
unsigned long lastCommandAsk = 0;
unsigned long lastLoRaPacketSent = 0;
int batteryLevel = 0;  // Almacenará el nivel de batería
unsigned long lastLoRaSend = 0;  // Tiempo del último paquete LoRa enviado
unsigned long timeBetweenPackets = 0;  
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // Tiempo entre paquetes
unsigned long lastOledUpdate = 0;  // Control de actualización OLED
unsigned long timeRx = 0;
unsigned long rxStartTime = 0;
//unsigned long lastPrintMillis = 0;  // Control de impresión en Serial Monitor
uint8_t receivedData[256] = {0};  // Buffer para datos del paquete
//char receivedData[30] = {0};
float encoderRatio = 0.0;
int encoderType = 0;
char runCommandInit[4] = {0};
char runCommand[5] = {0};
bool recepciona = false;
bool infoRecibida = false;

unsigned long lowPowerStartTime = 0; // Variable para guardar el tiempo de inicio en LOWPOWER



void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
 
  //########################## ENCODER ###############################
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP); // sets pin A as input with pull-up
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP); // sets pin B as input with pull-up
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterruptA, FALLING);

  VextON();
  display.init();
  display.setFont(ArialMT_Plain_16);
  Serial.println("OLED configurado");
  display.clear();
 
    state=STATE_TX;
}



void loop()
{
  //Serial.println(recepciona);
      unsigned long inicio = 0;
      unsigned long finall = 0;
  uint8_t requestData[1] = {1};
  currentMillis = millis();
  //Serial.println(currentMillis);
  if(infoRecibida){
        if (currentMillis - lastDisplayUpdate >= 1500) {
        updateOLED();
        lastDisplayUpdate = currentMillis;
      }
      // Actualizar el nivel de batería cada 30 segundos
      if (currentMillis - lastBatteryUpdate >= 20000) {
        batteryLevel = getBatteryLevel();
        lastBatteryUpdate = currentMillis;
      }}
  switch(state)
  {

  
    case STATE_TX:
     // delay(10);
      if(!infoRecibida){
     // Serial.println((int)requestData[0]);
      Radio.Send(requestData, 1);
     // Serial.println("TX AAAAA");
      //delay(50);
      recepciona = false;
      state = LOWPOWER;}
      else{
        
    // if(currentMillis - lastLoRaPacketSent >= 33){
     // Serial.println("Tiempo pasado:");
     // delay(1000);
        Serial.println(currentMillis - lastLoRaPacketSent);
        lastLoRaPacketSent = currentMillis;
        sendLoRaPacket();
      }//}
      //lowPowerStartTime = millis(); // Guardar tiempo de inicio de LOWPOWER
      break;

    case STATE_RX:
     // delay(10);
     // Serial.println("into RX mode");
      Radio.Rx(0);
      state = LOWPOWER;
      break;

    case LOWPOWER:
      //Serial.println("lowpower");
      //inicio = millis();
      Radio.IrqProcess();
      //finall = millis();
     // Serial.println(inicio - finall);
      if (!recepciona){
      //Radio.IrqProcess();
      rxStartTime = millis();
      recepciona = true;}
      //Serial.println("hago millis");
     // }else{
      if (millis() - rxStartTime >= 50){
        rxStartTime = millis();
        Serial.println("RX timeout, volviendo a TX");
        state = STATE_TX;}
      
      break;

    default:
      break;
  }
}


void OnTxDone( void )
{
 // Serial.println("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    //Radio.Sleep( );
    //Serial.println("TX Timeout......");
    state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  recepciona = true;
    Rssi=rssi;
    rxSize=size;
  //Serial.println("recibí algo");
  //rxDone = true;
  //Radio.Sleep();
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
  infoRecibida = true;
 // delay(10000);
    state=STATE_TX;
}

void sendLoRaPacket(){
  unsigned long inicio1 = 0;
  unsigned long finall1 = 0;
  inicio1 = millis();
  uint8_t buffer[7] = {0}; // Tamaño total: 1 byte para el ID, 4 bytes para _LeftEncoderTicks, 1 bytes para batteryLevel y 1 byte para checksum
  uint8_t bat8 = (uint8_t)batteryLevel;
  buffer[0] = 2;
  memcpy(buffer + 1, &_LeftEncoderTicks, sizeof(_LeftEncoderTicks));
  memcpy(buffer + 5, &bat8, sizeof(bat8));
  // Leer batteryLevel (1 byte - tipo `uint8_t`)
  buffer[6] = xorChecksum(buffer, sizeof(buffer)); 
  // Enviar los bytes directamente
  Radio.Send(buffer, sizeof(buffer));
  finall1 = millis();
  Serial.println(finall1 - inicio1);
  state = LOWPOWER;
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

uint8_t xorChecksum(uint8_t* buffer, int len){
  uint8_t sum = 0;
  for (int i = 0; i < 5; i++) {
      sum ^= buffer[i];
  }
  return sum;
}

int getBatteryLevel() {
  float voltage = (analogRead(batteryPin) / adcResolution) * referenceVoltage / voltageDividerRatio;
  int percentage = map(voltage * 100, 500, 800, 0, 100);
  return constrain(percentage, 0, 100);
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