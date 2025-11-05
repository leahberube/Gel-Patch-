#include "NAND_Micron.h"

const uint16_t BYTES_PER_PAGE  = 2048;
const uint16_t PAGES_PER_BLOCK = 64;
const uint32_t BLOCKS_TOTAL    = 2048;

static inline uint32_t rowAddress(uint32_t block, uint16_t page) {
  return ((block & 0x07FFUL) << 6) | (page & 0x3F);
}

void nandReset() {
  csLow(); spiXferByte(CMD_RESET); csHigh(); delay(2);
}

void nandWriteEnable() {
  csLow(); spiXferByte(CMD_WRITE_ENABLE); csHigh();
}

uint8_t nandGetFeature(uint8_t addr) {
  csLow();
  spiXferByte(CMD_GET_FEATURE);
  spiXferByte(addr);
  uint8_t val = spiXferByte(0);
  csHigh();
  return val;
}

void nandSetFeature(uint8_t addr, uint8_t val) {
  csLow();
  spiXferByte(CMD_SET_FEATURE);
  spiXferByte(addr);
  spiXferByte(val);
  csHigh();
}

void nandWaitReady() {
  while (nandGetFeature(REG_STATUS) & ST_OIP) delayMicroseconds(50);
}

bool nandBlockErase(uint32_t block) {
  nandWriteEnable();
  uint32_t row = rowAddress(block, 0);
  csLow();
  spiXferByte(CMD_BLOCK_ERASE);
  spiXferByte((row >> 16) & 0xFF);
  spiXferByte((row >> 8) & 0xFF);
  spiXferByte(row & 0xFF);
  csHigh();
  nandWaitReady();
  return !(nandGetFeature(REG_STATUS) & ST_P_FAIL);
}

/* ---- Page programming ---- */
bool startNewPage() {
  curCol = 0;
  return true;
}

bool pageAppendChunk(const uint8_t* data, uint16_t len) {
  if (curCol + len > BYTES_PER_PAGE) return false;
  nandWriteEnable();
  csLow();
  if (curCol == 0) {
    spiXferByte(CMD_PROGRAM_LOAD_X1);
    spiXferByte(0x00);
    spiXferByte(0x00);
  } else {
    spiXferByte(CMD_PROGRAM_LOAD_RANDOM_X1);
    spiXferByte((curCol >> 8) & 0xFF);
    spiXferByte(curCol & 0xFF);
  }
  for (uint16_t i = 0; i < len; i++) spiXferByte(data[i]);
  csHigh();
  curCol += len;
  return true;
}

bool finalizePageProgram() {
  if (curCol == 0) return true;
  nandWriteEnable();
  uint32_t row = rowAddress(curBlock, curPage);
  csLow();
  spiXferByte(CMD_PROGRAM_EXECUTE);
  spiXferByte((row >> 16) & 0xFF);
  spiXferByte((row >> 8) & 0xFF);
  spiXferByte(row & 0xFF);
  csHigh();
  nandWaitReady();
  bool ok = !(nandGetFeature(REG_STATUS) & ST_P_FAIL);
  if (ok) {
    curPage++;
    if (curPage >= PAGES_PER_BLOCK) {
      curPage = 0;
      curBlock++;
    }
  }
  return ok;
}

// void sendPageAsCSV(uint32_t block, uint16_t page) {
//   Serial.print("# Send page ");
//   Serial.print(block);
//   Serial.print(",");
//   Serial.println(page);
// }
