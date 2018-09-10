
#include <stdint.h>

unsigned int FlashRead(unsigned int addr);
void FlashUnlock(void);
void FlashEraseRow(uint16_t rowAddr);
void FlashWrite(unsigned int addr, unsigned int flash_data);


