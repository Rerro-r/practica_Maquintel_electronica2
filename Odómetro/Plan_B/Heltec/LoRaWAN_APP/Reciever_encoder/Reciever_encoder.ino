
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"


/////////////////////// LoRa //////////////////////////////////////////

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
#define BUFFER_SIZE                                 30 // Define the payload size here

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

int16_t rssi,rxSize;

bool lora_idle = true;

/////////////////// OLED /////////////////////////////////////////

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

////////////////// Variables traspasadas ////////////////////////

unsigned long previousMillis = 0;  // Tiempo entre paquetes
unsigned long lastOledUpdate = 0;  // Control de actualización OLED
unsigned long timeRx = 0;
unsigned long currentMillis = 0;
//unsigned long lastPrintMillis = 0;  // Control de impresión en Serial Monitor
uint8_t receivedData[7] = {0};  // Buffer para datos del paquete
//char receivedData[30] = {0};
uint8_t batteryLevel = 0;
uint8_t batteryLevelAnt = 0;
long leftEncoderTicks = 0;
long leftEncoderTicksAnt = 0;
float encoderRatio = 0.0;
int encoderType = 0;
char runCommandInit[4] = {0};
char runCommand[5] = {0};


void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  
  rssi=0;

  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.TxDone = OnTxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                              LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                              LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                              0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

  if(Radio.GetStatus()){
    Serial.println("Configuración LoRa establecida");
  }

  VextON();
  display.init();
  display.setFont(ArialMT_Plain_16);
  Serial.println("OLED configurado");
  delay(500);
  display.clear();

  Serial.println("Esperando datos por Serial...");
  String run_odometro_radio = ""; // Variable para almacenar el dato recibido
  // Esperar hasta que llegue un dato por serial
  while (run_odometro_radio.length() == 0) {
    // Mostrar mensaje de espera
    display.display(); // Mostrar mensaje en pantalla
    if (Serial.available() > 0) {
      run_odometro_radio = Serial.readString(); // Leer el dato como cadena
      display.clear(); // Limpiar pantalla solo cuando se reciben datos
      display.drawString(0, 0, "serial recibido");
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
  display.clear();
  display.drawString(0, 0, "serial: ");
  display.drawString(0, 20, run_odometro_radio);
  delay(1000); // Mostrar los resultados durante 1 segundos
  display.display();
}



void loop(){

  currentMillis = millis();  // Obtener tiempo actual

  if(lora_idle){
    lora_idle = false;
    Radio.Rx(0);
  }
  Radio.IrqProcess( );

}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  rssi=rssi;
  rxSize=size;
  memcpy(receivedData, payload, sizeof(receivedData));
  int indexQuestion = 0;
  indexQuestion = receivedData[0];  // Determinar tipo de pregunta

  if (indexQuestion == 1) {
    delay(200);
    // Enviar configuración del encoder en binario por LoRa
    int bufferSize = 1 + sizeof(runCommandInit) + sizeof(encoderType) + sizeof(encoderRatio);
    uint8_t buffer[bufferSize]; // Buffer de tamaño variable
    // Construir el paquete en el buffer
    buffer[0] = 1; // Confirmación de recepción (como uint8_t)
    memcpy(buffer + 1, &runCommandInit, sizeof(runCommandInit)); // Copia el string
    buffer[1 + sizeof(runCommandInit)] = encoderType; // Copia el tipo de encoder
    memcpy(buffer + 1 + sizeof(runCommandInit) + sizeof(encoderType), &encoderRatio, sizeof(encoderRatio)); // Copia el ratio del encoder
    Radio.Send(buffer, sizeof(buffer));
    Serial.println("info enviada");

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
  //Serial.println("solicitud pedida");
  Serial.println(currentMillis - timeRx);
  timeRx = currentMillis;
  lora_idle = true;
}

void OnTxDone( void )
{
  lora_idle = true;
}

void OnTxTimeout( void )
{
  lora_idle = true;
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

void handleSerialData() {
  String data = Serial.readString();
  if (data.length() > 0) {
    data.toCharArray(runCommand, sizeof(runCommand));
    // Mostrar el dato enviado y el estado en el display
    display.clear();
    display.drawString(0, 0, runCommand);
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
