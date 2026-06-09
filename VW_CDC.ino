#include <SPI.h>
#include <ESP_I2S.h>
#include <Arduino.h>
#include "BluetoothA2DPSink.h"

// --- KONFIGURACJA PINÓW ---
#define SPI_MOSI 21
#define SPI_MISO 18 // Pin nasłuchujący (Data Out z radia)
#define SPI_SCK  33
#define SPI_SS   5
#define DEVICE_NAME "ChangerBT"

I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);
SPIClass *vspi = NULL;

// --- ZMIENNE ODTWARZACZA ---
uint8_t trackno = 1;
uint8_t g_playMinutes = 0;
uint8_t g_playSeconds = 0;
uint32_t lastTickTime = 0;
uint8_t g_discLoad = 0x2E; 

// --- DEKODER VW (RING BUFFER) ---
#define VW_CAPBUFFER_SIZE 24
volatile uint8_t vw_capBuffer[VW_CAPBUFFER_SIZE];
volatile uint8_t vw_capPtr = 0;
volatile uint8_t vw_scanPtr = 0;
volatile bool vw_capBusy = false;
volatile uint8_t vw_capBit = 8;
volatile uint8_t vw_capBitPacket = 0;
volatile uint8_t vw_currentByte = 0;
volatile uint32_t vw_lastFallingEdge = 0;
volatile bool vw_measuringLow = false;

// Progi czasowe z vwcdpic (w mikrosekundach)
const uint32_t VW_START_THRESHOLD = 3200;
const uint32_t VW_HIGH_THRESHOLD  = 1248;
const uint32_t VW_LOW_THRESHOLD   = 256;
const uint8_t  VW_PKTSIZE         = 32;

// --- KONWERSJA BCD ---
uint8_t toBCD(uint8_t val) {
    if (val > 99) val = 99;
    return ((val / 10) << 4) | (val % 10);
}

// --- PRZERWANIE (ODCZYT PRZYCISKÓW) ---
void IRAM_ATTR vw_dataout_isr() {
    uint32_t now = micros();
    bool level = digitalRead(SPI_MISO);
    
    if (!level) {
        // ZBOCZE OPADAJĄCE
        vw_lastFallingEdge = now;
        vw_measuringLow = true;
    } else {
        // ZBOCZE NARASTAJĄCE
        if (!vw_measuringLow) return; 
        
        uint32_t lowDuration = now - vw_lastFallingEdge;
        vw_measuringLow = false;
        
        if (lowDuration < VW_LOW_THRESHOLD) return; // Szum
        
        // Start nowej paczki
        if (lowDuration >= VW_START_THRESHOLD) {
            vw_capBusy = true;
            vw_capBitPacket = VW_PKTSIZE;
            vw_capBit = 8;
            vw_currentByte = 0;
            return; 
        }
        
        if (!vw_capBusy || vw_capBitPacket == 0) return;
        
        uint8_t bitValue = (lowDuration >= VW_HIGH_THRESHOLD) ? 1 : 0;
        
        vw_currentByte <<= 1;
        if (bitValue) vw_currentByte |= 0x01;
        
        vw_capBit--;
        vw_capBitPacket--;
        
        if (vw_capBit == 0) {
            vw_capBuffer[vw_capPtr] = vw_currentByte;
            vw_capPtr = (vw_capPtr + 1) % VW_CAPBUFFER_SIZE;
            vw_capBit = 8;
            vw_currentByte = 0;
        }
        
        if (vw_capBitPacket == 0) {
            vw_capBusy = false;
        }
    }
}

// --- ANALIZA PACZEK Z PRZYCISKAMI ---
void vw_scanCommandBytes() {
    static uint32_t lastActionTime = 0;
    uint32_t currentTime = millis();

    while (vw_scanPtr != vw_capPtr) {
        uint8_t byte1 = vw_capBuffer[vw_scanPtr];
        if (byte1 != 0x53) {
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        uint8_t available = (vw_capPtr >= vw_scanPtr) ? 
                            (vw_capPtr - vw_scanPtr) : 
                            (VW_CAPBUFFER_SIZE - vw_scanPtr + vw_capPtr);
        
        if (available < 4) return; // Czekaj na resztę
        
        uint8_t byte2 = vw_capBuffer[(vw_scanPtr + 1) % VW_CAPBUFFER_SIZE];
        uint8_t byte3 = vw_capBuffer[(vw_scanPtr + 2) % VW_CAPBUFFER_SIZE];
        uint8_t byte4 = vw_capBuffer[(vw_scanPtr + 3) % VW_CAPBUFFER_SIZE];
        
        if (byte2 != 0x2C || (uint8_t)(byte3 + byte4) != 0xFF || (byte3 & 0x03) != 0) {
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        uint8_t cmdcode = byte3;
        
        // Anti-bounce dla przycisków (1 sekunda)
        if (currentTime - lastActionTime > 1000) {
            if (cmdcode == 0xF8) {
                trackno = (trackno >= 99) ? 1 : trackno + 1;
                a2dp_sink.next();
                g_playSeconds = 0;
                g_playMinutes = 0;
                lastActionTime = currentTime;
            } 
            else if (cmdcode == 0x78) {
                trackno = (trackno <= 1) ? 99 : trackno - 1;
                a2dp_sink.previous();
                g_playSeconds = 0;
                g_playMinutes = 0;
                lastActionTime = currentTime;
            }
        }
        vw_scanPtr = (vw_scanPtr + 4) % VW_CAPBUFFER_SIZE;
    }
}

// --- WYSYŁANIE RAMKI SPI ---
void cdc_sendSpiPacket(const uint8_t frame[8]) {
    vspi->beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE1));
    for (int i = 0; i < 8; ++i) {
        vspi->transfer(frame[i]);
        delayMicroseconds(874); // Wymagany odstęp między bajtami!
    }
    vspi->endTransaction();
}

TaskHandle_t RadioTaskHandle;

// --- GŁÓWNA MASZYNA STANÓW RADIA ---
void radioTaskCode(void * pvParameters) {
    enum { ST_IDLE_THEN_PLAY, ST_INIT_PLAY, ST_PLAY_LEAD_IN, ST_PLAY };
    int state = ST_IDLE_THEN_PLAY;
    int stateCounter = -20;
    uint32_t prevMs = millis();

    for (;;) {
        vw_scanCommandBytes();
        
        uint32_t now = millis();
        
        // Tykanie zegarka tylko w trybie PLAY
        if (state == ST_PLAY && (now - lastTickTime >= 1000)) {
            lastTickTime = now;
            g_playSeconds++;
            if (g_playSeconds >= 60) {
                g_playSeconds = 0;
                g_playMinutes++;
                if (g_playMinutes >= 100) g_playMinutes = 0;
            }
        }
        
        // Ramka leci co 50ms (20Hz)
        if (now - prevMs >= 50) { 
            prevMs = now;
            uint8_t disc = 1;
            
            if (state == ST_IDLE_THEN_PLAY) {
                uint8_t idle[8] = {0x74, 0xBE, (uint8_t)(0xFF - trackno), 0xFF, 0xFF, 0xFF, 0x8F, 0x7C};
                cdc_sendSpiPacket(idle);
                
                stateCounter++;
                if (stateCounter >= 0) {
                    state = ST_INIT_PLAY;
                    stateCounter = -24;
                    g_discLoad = 0x2E;
                }
            }
            else if (state == ST_INIT_PLAY) {
                bool isAnnounce = ((stateCounter & 1) == 0);
                if (isAnnounce) {
                    uint8_t frame[8] = {0x34, g_discLoad, (uint8_t)(0xFF - 0x99), (uint8_t)(0xFF - 0x99), (uint8_t)(0xFF - 0x59), 0xB7, 0xFF, 0x3C};
                    cdc_sendSpiPacket(frame);
                    if (g_discLoad == 0x29) g_discLoad = 0x2E; else g_discLoad--;
                } else {
                    uint8_t frame[8] = {0x34, 0xBE, (uint8_t)(0xFF - trackno), 0xFF, 0xFF, 0xFF, 0xEF, 0x3C};
                    cdc_sendSpiPacket(frame);
                }
                
                stateCounter++;
                if (stateCounter >= 0) {
                    state = ST_PLAY_LEAD_IN;
                    stateCounter = -10;
                }
            }
            else if (state == ST_PLAY_LEAD_IN) {
                bool isAnnounce = ((stateCounter & 1) == 0);
                if (isAnnounce) {
                    uint8_t frame[8] = {0x34, (uint8_t)((disc & 0x0F) | 0x20), (uint8_t)(0xFF - 0x99), (uint8_t)(0xFF - 0x99), (uint8_t)(0xFF - 0x59), 0xB7, 0xFF, 0x3C};
                    cdc_sendSpiPacket(frame);
                } else {
                    uint8_t frame[8] = {0x34, 0xBE, (uint8_t)(0xFF - trackno), 0xFF, 0xFF, 0xFF, 0xAE, 0x3C};
                    cdc_sendSpiPacket(frame);
                }
                
                stateCounter++;
                if (stateCounter >= 0) {
                    state = ST_PLAY;
                    lastTickTime = millis(); // Reset zegarka przy starcie PLAY
                }
            }
            else if (state == ST_PLAY) {
                uint8_t minBCD = toBCD(g_playMinutes);
                uint8_t secBCD = toBCD(g_playSeconds);
                uint8_t frame[8] = {0x34, 0xBE, (uint8_t)(0xFF - toBCD(trackno)), (uint8_t)(0xFF - minBCD), (uint8_t)(0xFF - secBCD), 0x00, 0xCF, 0x3C};
                cdc_sendSpiPacket(frame);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Luźna pętla zadania
    }
}

void setup() {
    vspi = new SPIClass(HSPI);
    vspi->begin(SPI_SCK, -1, SPI_MOSI, SPI_SS); 
    
    pinMode(SPI_SS, OUTPUT);
    digitalWrite(SPI_SS, HIGH);

    pinMode(SPI_MISO, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPI_MISO), vw_dataout_isr, CHANGE);

    xTaskCreatePinnedToCore(
        radioTaskCode,    
        "RadioTask",      
        8192,             
        NULL,             
        1,                
        &RadioTaskHandle, 
        1                 
    );

    i2s.setPins(26, 25, 22);
    i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);  
    a2dp_sink.set_volume(128);
    a2dp_sink.set_auto_reconnect(true); 
    a2dp_sink.start(DEVICE_NAME);
}

void loop() {
    vTaskDelay(portMAX_DELAY); 
}
