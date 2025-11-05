#ifndef NAND_MICRON_H
#define NAND_MICRON_H

#include <Arduino.h>

/* -------- NAND commands -------- */
#define CMD_GET_FEATURE            0x0F
#define CMD_SET_FEATURE            0x1F
#define CMD_RESET                  0xFF
#define CMD_WRITE_ENABLE           0x06
#define CMD_BLOCK_ERASE            0xD8
#define CMD_PAGE_READ              0x13
#define CMD_READ_FROM_CACHE_X1     0x03
#define CMD_PROGRAM_LOAD_X1        0x02
#define CMD_PROGRAM_LOAD_RANDOM_X1 0x84
#define CMD_PROGRAM_EXECUTE        0x10

/* -------- Feature / status registers -------- */
#define REG_BLOCK_LOCK     0xA0
#define REG_CONFIGURATION  0xB0
#define REG_STATUS         0xC0
#define CFG_ECC_EN         0x10
#define ST_OIP             0x01
#define ST_P_FAIL          0x08

/* -------- NAND geometry -------- */
extern const uint16_t BYTES_PER_PAGE;
extern const uint16_t PAGES_PER_BLOCK;
extern const uint32_t BLOCKS_TOTAL;

/* -------- SPI interface (implemented in .ino) -------- */
extern uint8_t spiXferByte(uint8_t v);
extern void csLow();
extern void csHigh();

/* -------- Function prototypes -------- */
void nandReset();
void nandWriteEnable();
uint8_t nandGetFeature(uint8_t addr);
void nandSetFeature(uint8_t addr, uint8_t val);
void nandWaitReady();
bool nandBlockErase(uint32_t block);
bool startNewPage();
bool pageAppendChunk(const uint8_t* data, uint16_t len);
bool finalizePageProgram();
void sendPageAsCSV(uint32_t block, uint16_t page);

/* -------- Externs for page tracking -------- */
extern uint32_t curBlock;
extern uint16_t curPage;
extern uint16_t curCol;

#endif
