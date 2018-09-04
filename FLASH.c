/*
 * File:   MemoryFcn.c
 * Author: Evgeny
 *
 * Created on 16 ��� 2018 �., 10:33
 */


#include <xc.h>
#include "FLASH.h"
#include <stdint.h>


unsigned int FlashRead(unsigned int addr) {//good
    PMCON1bits.CFGS = 0;
    PMADRH = (unsigned char) (addr >> 8);
    PMADRL = (unsigned char) (addr);
    PMCON1bits.RD = 1;

    return ((uint16_t) PMDATH << 8 | PMDATL);
}

void FlashUnlock(void) {//good
    PMCON2 = 0x55;
    PMCON2 = 0xAA;
    PMCON1bits.WR = 1;
}

void FlashEraseRow(unsigned int rowAddr) {//good
    PMCON1bits.CFGS = 0;
    PMADRH = (unsigned char) (rowAddr >> 8);
    PMADRL = (unsigned char) (rowAddr);
    PMCON1bits.FREE = 1;
    PMCON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    FlashUnlock();
    INTCONbits.GIE = 1;
    PMCON1bits.WREN = 0;
}

void FlashWrite(unsigned int addr, unsigned int flash_data) {//good
    PMCON1bits.CFGS = 0;
    PMADRH = (unsigned char) (addr >> 8);
    PMADRL = (unsigned char) (addr);
    PMCON1bits.FREE = 0;
    PMCON1bits.LWLO = 1;
    PMCON1bits.WREN = 1;
    PMDATH = (unsigned char) (flash_data >> 8);
    PMDATL = (unsigned char) flash_data;
    PMCON1bits.LWLO = 0;
    INTCONbits.GIE = 0;
    FlashUnlock();
    INTCONbits.GIE = 1;
    PMCON1bits.WREN = 0;
}




