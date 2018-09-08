
#include <stdint.h>

struct {
    uint8_t State;
    uint8_t StateTemp;
    uint8_t Tick100ms;
    uint8_t FirstCmdSent;
} typedef KeyState;

uint8_t Init_TypeFromFlash(const uint16_t* type);
uint8_t Init_TxStatusFromFlash(const uint16_t* txStatus);
void KeyOffHandler(KeyState* key, uint8_t chn, uint8_t cmd, uint8_t* nooData);

void KeyLongHandler(KeyState* key, uint8_t chn, uint8_t cmd, uint8_t ticksToTrigger, uint8_t* nooData);
