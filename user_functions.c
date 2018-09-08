
#include <xc.h>
#include <pic16lf1503.h>
#include <stdint.h>
#include "user_functions.h"
#include "noolite.h"
#define _XTAL_FREQ 8000000

enum {
    CMD_OFF = 0, CMD_Bright_Down = 1, CMD_ON = 2,
    CMD_Bright_Up = 3, CMD_Switch = 4, CMD_Bright_Back = 5,
    CMD_Load_Preset = 7, CMD_Save_Preset = 8, CMD_Unbind = 9,
    CMD_Stop_Reg = 10, CMD_Bind = 15, CMD_Test_Result = 22
};

//ADC channels

enum {
    Battery = 31
};

uint8_t Init_TypeFromFlash(const uint16_t* type) {
    if (((type[0] >> 8) == 0x5A) && ((type[0] & 0xFF) < 4)) {
        return (type[0] & 0xFF);
    } else {
        return 0;
    }
}

uint8_t Init_TxStatusFromFlash(const uint16_t* txStatus) {
    for (uint8_t cellNum = 0; cellNum < 8; cellNum++) {
        if (txStatus[cellNum] == 0xFFFF) {
            if (cellNum > 0) {
                if (((txStatus[cellNum - 1] >> 8) == 0x5A) && ((txStatus[cellNum - 1] & 0xFF) < 3)) {
                    return txStatus[cellNum - 1];
                } else {
                    return 0;
                }
            } else {
                return 0;
            }
        } else {
            if (cellNum == 7) {
                if (((txStatus[cellNum] >> 8) == 0x5A) && ((txStatus[cellNum] & 0xFF) < 3)) {
                    return txStatus[cellNum];
                } else {
                    return 0;
                }
            }
        }
    }
    return 0;
}

void KeyOffHandler(KeyState* key, uint8_t chn, uint8_t cmd, uint8_t* nooData) {
    if (key->State == 0) {
        if (key->Tick100ms < 10) { //key pressed SHORT
            noolite_send(chn, cmd, 0, &nooData[0]);
        } else { //key pressed LONG
            noolite_send(chn, CMD_Stop_Reg, 0, &nooData[0]);
            __delay_ms(15);
            noolite_send(chn, CMD_Stop_Reg, 0, &nooData[0]);
            __delay_ms(15);
            noolite_send(chn, CMD_Stop_Reg, 0, &nooData[0]);
            key->FirstCmdSent = 0;
        }
    }
}

void KeyLongHandler(KeyState* key, uint8_t chn, uint8_t cmd, uint8_t ticksToTrigger, uint8_t* nooData) {
    if (key->State != 0) {
        if (key->Tick100ms > ticksToTrigger) {
            if (key->FirstCmdSent == 0) {
                noolite_send(chn, cmd, 0, &nooData[0]);
                key->FirstCmdSent = 1;
            }
        }
        key->Tick100ms++;
    } else {
        key->Tick100ms = 0;
    }
}

unsigned int GetAdcValue(uint8_t channel) {
    ADCON0bits.CHS = (uint8_t) (channel & 0x1F); //установка канала ANx для измерения 

    if (channel == Battery) {
        FVRCONbits.FVREN = 1;
        FVRCONbits.ADFVR = 0b01; //ADC = 1.024 / Vdd
    }

    ADCON0bits.ADON = 1;
    __delay_us(55);

    ADCON0bits.GO_nDONE = 1; //start conversion
    while (ADCON0bits.GO_nDONE) {
    } //wait for conv. complete

    //10 bit res
    unsigned int result;
    result = (uint16_t) ((ADRESH << 8) | ADRESL);

    //8 bit res
    //unsigned char result;
    //result = ADRESH;

    ADCON0bits.ADON = 0;
    FVRCON = 0;
    PIR1bits.ADIF = 0;

    return result;
}