#include <SPI.h>
#include <ESP_I2S.h>
#include <Arduino.h>
#include "BluetoothA2DPSink.h"

#define MODE_PLAY 0xFF
#define PLAY_NEXT 0x3F
#define PLAY_PREV 0x3D

#define SPI_MOSI 21
#define SPI_MISO 18
#define SPI_SCK  33
#define SPI_SS   5

#define CD_TIME 1000
#define DEVICE_NAME "ChangerBT"

I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);

SPIClass *vspi = NULL;

volatile uint32_t edges[200];
volatile bool levels[200];
volatile int edgeIndex = 0;
volatile bool frameReady = false;
volatile uint32_t lastEdgeTime = 0;

uint8_t trackno = 1;


void IRAM_ATTR handleEdge() {
  uint32_t now = micros();
  bool level = digitalRead(SPI_MISO);

  if (frameReady) return;

  if (edgeIndex < 200) {
    edges[edgeIndex] = now;
    levels[edgeIndex] = level;
    edgeIndex++;
  }

  lastEdgeTime = now;
}

void analyzeFrame() {
  noInterrupts();
  int count = edgeIndex;
  uint32_t times[200];
  bool states[200];
  for (int i = 0; i < count; i++) { 
    times[i] = edges[i]; 
    states[i] = levels[i]; 
  } 

  edgeIndex = 0; 
  frameReady = false; 
  interrupts(); 

  if (count < 10) { 
    return; 
  } 

  uint8_t bits[6] = {0}; 
  int bitCounter = 0; 
  for (int i = 1; i < count; i += 2) { 
    if (i + 1 >= count) break; 
    uint32_t highTime = times[i] - times[i - 1]; 
    uint32_t lowTime = (i + 1 < count) ? times[i + 1] - times[i] : 0;
    uint32_t total = highTime + lowTime; 

    bool bitVal = (total < 1500); 

    int byteIndex = bitCounter / 8; 
    int bitIndex = bitCounter % 8; 
    if (bitVal) bits[byteIndex] |= (1 << bitIndex); 

    bitCounter++; 
    if (bitCounter >= 48) break; 
  } 

  static uint32_t lastActionTime = 0;
  uint32_t currentTime = millis();
  
  // Ignorowanie komend przez CD_TIME ms od ostatniego przełączenia
  if (currentTime - lastActionTime < CD_TIME) {
      return; 
  }

  for (int i = 0; i < 6; i++) { 
    if (bits[i] == PLAY_NEXT) { 
      trackno++; 
      a2dp_sink.next();
      lastActionTime = currentTime;
      break; 
    } else if (bits[i] == PLAY_PREV) { 
      if (trackno > 1) trackno--; 
      a2dp_sink.previous();
      lastActionTime = currentTime;
      break; 
    }
  }
}

uint8_t transmit_msg(const uint8_t *in) {
  digitalWrite(SPI_SS, LOW);
  vspi->beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE1));
  for (int i = 0; i < 8; i++) {
    vspi->transfer(in[i]);
    delay(2);
  }
  vspi->endTransaction();
  digitalWrite(SPI_SS, HIGH);
  return 0;
}

TaskHandle_t RadioTaskHandle;

// Zadanie obsługujące komunikację z radiem
void radioTaskCode(void * pvParameters) {
  for (;;) {
    uint8_t cdno = 1;
    uint8_t cd = 0xFF ^ cdno;
    uint8_t track = 0xFF ^ trackno;
    uint8_t msg[] = {0x34, cd, track, 0xFF, 0xFF, MODE_PLAY, 0xCF, 0x3C};
    
    if (trackno == 10) {
      trackno = 1;
    }

    transmit_msg(msg);

    if (!frameReady && edgeIndex > 10 && (micros() - lastEdgeTime > 30000)) {
      frameReady = true;
    }

    if (frameReady) {
      analyzeFrame();
    }

    // Zamiennik delay() w FreeRTOS - nie blokuje innych procesów na rdzeniu
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void setup() {
  vspi = new SPIClass(HSPI);
  vspi->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);

  pinMode(SPI_MISO, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPI_MISO), handleEdge, CHANGE);

  xTaskCreatePinnedToCore(
    radioTaskCode,    // Funkcja zadania
    "RadioTask",      // Nazwa zadania
    8192,             // Rozmiar stosu w bajtach
    NULL,             // Parametry wejściowe
    1,                // Priorytet zadania (1 - standardowy)
    &RadioTaskHandle, // Uchwyt do zarządzania zadaniem
    1                 // Przypisanie do rdzenia 1
  );

  i2s.setPins(26, 25, 22);
    // Inicjalizacja I2S (format standardowego audio CD)
  i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);  
    // Przypisanie początkowej głośności
  a2dp_sink.set_volume(128);
  a2dp_sink.set_auto_reconnect(true); 
  a2dp_sink.start(DEVICE_NAME);
}

void loop() {
  // Główna pętla pozostaje pusta, usypiamy ją bez końca.
  // Cała praca odbywa się w radioTaskCode i przerwaniach.
  vTaskDelay(portMAX_DELAY); 
}
