#include <SPI.h>
#include <btAudio.h>
#include <Arduino.h>
#include "BluetoothA2DPSink.h"

#define MODE_PLAY 0xFF
#define PLAY_NEXT 0x3F
#define PLAY_PREV 0x3D

#define SPI_MOSI 21
#define SPI_MISO 18
#define SPI_SCK  33
#define SPI_SS   5

BluetoothA2DPSink a2dp_sink;

SPIClass *vspi = NULL;

volatile uint32_t edges[200];
volatile bool levels[200];
volatile int edgeIndex = 0;
volatile bool frameReady = false;
volatile uint32_t lastEdgeTime = 0;

uint8_t trackno = 1;

void sendAVRCPCommand(uint8_t cmd) {
  esp_avrc_ct_send_passthrough_cmd(0, cmd, ESP_AVRC_PT_CMD_STATE_PRESSED);
  delay(50);
  esp_avrc_ct_send_passthrough_cmd(0, cmd, ESP_AVRC_PT_CMD_STATE_RELEASED);
}

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

    bool bitVal = (total < 1500); // 1 ~1.2ms, 0 ~1.76ms

    int byteIndex = bitCounter / 8;
    int bitIndex = bitCounter % 8;
    if (bitVal) bits[byteIndex] |= (1 << bitIndex);

    bitCounter++;
    if (bitCounter >= 48) break;
  }

  for (int i = 0; i < 6; i++) {
    if (bits[i] == PLAY_NEXT) {
      trackno++;
      sendAVRCPCommand(ESP_AVRC_PT_CMD_FORWARD);
      break;
    } else if (bits[i] == PLAY_PREV) {
      if (trackno > 1) trackno--;
      sendAVRCPCommand(ESP_AVRC_PT_CMD_BACKWARD);
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

void setup() {
  pinMode(SPI_MISO, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPI_MISO), handleEdge, CHANGE);

  vspi = new SPIClass(VSPI);
  vspi->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);

  i2s_pin_config_t pins = {
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = 22,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(pins);
  a2dp_sink.start("Radio VW");

}

void loop() {
  uint8_t cdno = 1;
  uint8_t cd = 0xFF ^ cdno;
  uint8_t track = 0xFF ^ trackno;
  uint8_t msg[] = {0x34, cd, track, 0xFF, 0xFF, MODE_PLAY, 0xCF, 0x3C};

  if (trackno ==10)
  {
    trackno = 1;
  }

  transmit_msg(msg);

  if (!frameReady && edgeIndex > 10 && (micros() - lastEdgeTime > 30000)) {
    frameReady = true;
  }

  if (frameReady) {
    analyzeFrame();
  }

  delay(100);
}
