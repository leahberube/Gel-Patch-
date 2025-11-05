#include <Wire.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "AD5933.h"
#include "NAND_Micron.h"

/* -------- PIN MAP (unchanged) -------- */
#define SOFTSPI_SCK_PIN   17
#define SOFTSPI_MOSI_PIN  11
#define SOFTSPI_MISO_PIN  12
#define SOFTSPI_CS_PIN    16
#define SOFTSPI_DELAY_US  5

/* -------- AD5933 configuration -------- */
#define START_FREQ    1000
#define FREQ_INCR     1
#define NUM_INCR      1
#define REF_RESIST    20000
#define SAMPLE_DELAY  50         // 20 Hz
#define BURST_PERIOD  120000UL   // 2 min

/* -------- Data record -------- */
struct ImpRec {
  uint32_t ms;
  float    impedance;
  uint8_t  reserved[8];
};

/* -------- Globals -------- */
AD5933 ad5933;
double gain[NUM_INCR + 1];
int phase[NUM_INCR + 1];
uint32_t curBlock = 0;
uint16_t curPage  = 0;
uint16_t curCol   = 0;
uint32_t lastSentBlock = 0;
uint16_t lastSentPage  = 0;
unsigned long startMillis = 0;
unsigned long lastBurst   = 0;

/* -------- BLE -------- */
const char * localName = "Impedance Patch";
BLEPeripheral blePeriph;
BLEService impedanceService("180C");
BLECharacteristic dataChar("2A56", BLERead | BLENotify, 20);   // 20-byte chunks
char bleBuffer[20];

/* -------- SoftSPI helpers (for NAND) -------- */
void csLow()  { digitalWrite(SOFTSPI_CS_PIN, LOW); }
void csHigh() { digitalWrite(SOFTSPI_CS_PIN, HIGH); }

uint8_t spiXferByte(uint8_t v) {
  uint8_t r = 0;
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SOFTSPI_MOSI_PIN, (v & 0x80) ? HIGH : LOW);
    delayMicroseconds(SOFTSPI_DELAY_US);
    digitalWrite(SOFTSPI_SCK_PIN, HIGH);
    delayMicroseconds(SOFTSPI_DELAY_US);
    r <<= 1;
    if (digitalRead(SOFTSPI_MISO_PIN)) r |= 1;
    digitalWrite(SOFTSPI_SCK_PIN, LOW);
    delayMicroseconds(SOFTSPI_DELAY_US);
    v <<= 1;
  }
  return r;
}

/* -------- Raw NAND page read -------- */
bool readPageRaw(uint32_t block, uint16_t page, uint8_t *buf) {
  uint32_t pageAddr = (block * PAGES_PER_BLOCK) + page;
  csLow();
  spiXferByte(0x13); // PAGE READ
  spiXferByte((pageAddr >> 16) & 0xFF);
  spiXferByte((pageAddr >> 8) & 0xFF);
  spiXferByte(pageAddr & 0xFF);
  csHigh();
  delayMicroseconds(50);
  csLow();
  spiXferByte(0x03);
  spiXferByte(0x00); spiXferByte(0x00); spiXferByte(0x00);
  for (uint16_t i=0; i<BYTES_PER_PAGE; i++) buf[i]=spiXferByte(0x00);
  csHigh();
  return true;
}

/* -------- Setup -------- */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(SOFTSPI_SCK_PIN, OUTPUT);
  pinMode(SOFTSPI_MOSI_PIN, OUTPUT);
  pinMode(SOFTSPI_MISO_PIN, INPUT);
  pinMode(SOFTSPI_CS_PIN, OUTPUT);
  digitalWrite(SOFTSPI_CS_PIN, HIGH);
  digitalWrite(SOFTSPI_SCK_PIN, LOW);
  digitalWrite(SOFTSPI_MOSI_PIN, LOW);

  Serial.println(F("\n# Initializing NAND + AD5933..."));
  nandReset();
  nandWriteEnable();
  nandSetFeature(REG_BLOCK_LOCK, 0x00);
  uint8_t cfg = nandGetFeature(REG_CONFIGURATION);
  nandWriteEnable();
  nandSetFeature(REG_CONFIGURATION, cfg | CFG_ECC_EN);
  nandBlockErase(0);
  startNewPage();
  Serial.println(F("# NAND ready."));

  if (!(ad5933.reset() &&
        ad5933.setInternalClock(true) &&
        ad5933.setStartFrequency(START_FREQ) &&
        ad5933.setIncrementFrequency(FREQ_INCR) &&
        ad5933.setNumberIncrements(NUM_INCR) &&
        ad5933.setPGAGain(PGA_GAIN_X1))) {
    Serial.println(F("# AD5933 init failed!"));
    while(1);
  }
  ad5933.setSettlingCycles(1);
  if (!ad5933.calibrate(gain, phase, REF_RESIST, NUM_INCR + 1)) {
    Serial.println(F("# Calibration failed!"));
    while(1);
  }

  // ---- BLE ----
  blePeriph.setDeviceName(localName);
  blePeriph.setLocalName(localName);
  blePeriph.setAdvertisedServiceUuid(impedanceService.uuid());
  blePeriph.addAttribute(impedanceService);
  blePeriph.addAttribute(dataChar);
  blePeriph.begin();
  Serial.println(F("# BLE advertising started."));

  startMillis = millis();
  lastBurst   = millis();
  Serial.println(F("# System ready. Logging started.\n"));
}

/* -------- Main Loop -------- */
void loop() {
  blePeriph.poll();

  int real[NUM_INCR + 1];
  int imag[NUM_INCR + 1];

  // ---- sample & log ----
  if (ad5933.frequencySweep(real, imag, NUM_INCR + 1)) {
    double mag = sqrt((double)real[0]*real[0] + (double)imag[0]*imag[0]);
    double impedance = 1.0 / (mag * gain[0]);

    ImpRec rec;
    rec.ms = millis() - startMillis;
    rec.impedance = impedance;
    memset(rec.reserved, 0xFF, sizeof(rec.reserved));

    if (curCol + sizeof(ImpRec) > BYTES_PER_PAGE) {
      finalizePageProgram();
      startNewPage();
    }
    pageAppendChunk((uint8_t*)&rec, sizeof(rec));
  }
  ad5933.setControlMode(0xA0);
  delay(SAMPLE_DELAY);

  // ---- every 2 min: send burst via BLE ----
  if (millis() - lastBurst >= BURST_PERIOD) {
    lastBurst = millis();
    finalizePageProgram();

    Serial.println(F("\n# --- 2-Minute BLE Burst ---"));
    uint8_t buf[BYTES_PER_PAGE];
    uint32_t b = lastSentBlock;
    uint16_t p = lastSentPage;

    while (true) {
      readPageRaw(b, p, buf);
      for (uint16_t offset=0; offset<BYTES_PER_PAGE; offset+=sizeof(ImpRec)) {
        ImpRec *rec = (ImpRec*)(buf + offset);
        if (rec->ms == 0xFFFFFFFF) break;
        if (rec->impedance <= 0 || rec->impedance > 1e6) continue;

        // --- Format time as HH:MM:SS instead of seconds ---
        uint32_t totalSeconds = rec->ms / 1000;
        uint8_t hours   = (totalSeconds / 3600) % 24;
        uint8_t minutes = (totalSeconds / 60) % 60;
        uint8_t seconds = totalSeconds % 60;

        snprintf(bleBuffer, sizeof(bleBuffer),
                 "%02u:%02u:%02u,%.3f",
                 hours, minutes, seconds, rec->impedance);

        dataChar.setValue((const unsigned char*)bleBuffer, strlen(bleBuffer));
        blePeriph.poll();   // keep link alive

        // optional: also mirror to Serial
        Serial.println(bleBuffer);

        delay(5);           // small gap between packets
      }

      p++;
      if (p >= PAGES_PER_BLOCK) { p = 0; b++; }
      if (b > curBlock || (b == curBlock && p >= curPage)) break;
    }

    lastSentBlock = curBlock;
    lastSentPage  = curPage;
    curPage++;
    if (curPage >= PAGES_PER_BLOCK) {
      curPage = 0; curBlock++;
      if (curBlock >= BLOCKS_TOTAL) curBlock = 0;
      nandBlockErase(curBlock);
    }
    startNewPage();
    Serial.println(F("# --- End of BLE Burst ---\n"));
  }
}
