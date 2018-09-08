/*
 * File:   main.c
 * Author: Evgeny
 *
 * Created on 31 августа 2017 г., 12:27
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = ON      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config BOREN = NSLEEP   // Brown-out Reset Enable (Brown-out Reset enabled while running and disabled in Sleep)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON       // Low-Power Brown Out Reset (Low-Power BOR is enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#pragma config IDLOC0 = 0x00
#pragma config IDLOC1 = 0x07
#pragma config IDLOC2 = 0x00
#pragma config IDLOC3 = 0x07

#include <pic16lf1503.h>
#include <xc.h>
#include "periph_config.h"
#include "FLASH.h"
#include "noolite.h"
#include"user_functions.h"
#include <stdint.h>
#define _XTAL_FREQ 8000000

//cmds

enum {
    CMD_OFF = 0, CMD_Bright_Down = 1, CMD_ON = 2,
    CMD_Bright_Up = 3, CMD_Switch = 4, CMD_Bright_Back = 5,
    CMD_Load_Preset = 7, CMD_Save_Preset = 8, CMD_Unbind = 9,
    CMD_Stop_Reg = 10, CMD_Bind = 15, CMD_Test_Result = 22
};

//keys at PORTA

enum {
    A_Pressed = 0x02,
    B_Pressed = 0x01,
    C_Pressed = 0x04,
    D_Pressed = 0x10
};

enum {
    All_Pressed = 0x17
};

//dev. service modes

enum {
    BIND_ACTIVE = 0x01,
    UNBIND_ACTIVE = 0x02,
    MODE_CHANGE_ACTIVE = 0x04,
    GO_OFF = 0x08
};

enum {
    OFF = 0,
    ON = 1
};

//channels for keys

enum {
    A = 0, B = 1, C = 2, D = 3, CD = 4
};

//ADC channels

enum {
    Battery = 31
};

//CONFIG-------------------------------------------------------------------------------------------------------------------
#define  AlarmBatteryVoltage 2.5    //значение напряжения батареи ниже которого происходит сигнализация
const uint16_t AlarmBattVoltage = (unsigned int) (1.024 / AlarmBatteryVoltage * 1023); //419
//--------------------------------------------------------------------------------------------------------------------------

#define LED         LATCbits.LATC3
#define RF          LATAbits.LATA5
#define VddLatch    LATCbits.LATC5

//Rf--------------------------------------------------------------------------------
extern unsigned char tx_status; //переменная статуса для передачи
unsigned char noo_send_data[4] = {0, 0, 0, 0}; //массив с передаваемыми данными по РК


uint8_t LedPulseTick_100ms = 0;


uint8_t tick3_100ms = 0;

//vars in FLASH adr

enum {
    TYPE_ADR = 0x7C0,
    TX_STATUS_ADR = 0x7D0
};
const uint16_t Type[] @ TYPE_ADR = {
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t TxStatus[] @ TX_STATUS_ADR = {
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

uint8_t DevType = 0;
uint8_t DevMode = 0;

KeyState Keys[5];

uint8_t SkipHandling = 0;
uint8_t OffDelayTicks_100ms = 0;

void main() {
    Init_IO();
    Init_CLK();
    Init_ADC();
    RF_Init();

    VddLatch = 1;
    DevType = Init_TypeFromFlash(&Type[0]);
    tx_status = Init_TxStatusFromFlash(&TxStatus[0]);

    INTCONbits.PEIE = 1; //enable peripheral interrupts
    INTCONbits.GIE = 1; //enable interrupt
    while (1) {
        //Bind combination (C+D) pressed  and not in BIND mode
        Keys[A].State = (uint8_t) (PORTA & A_Pressed);
        Keys[B].State = (uint8_t) (PORTA & B_Pressed);
        Keys[C].State = (uint8_t) (PORTA & C_Pressed);
        Keys[D].State = (uint8_t) (PORTA & D_Pressed);
        Keys[CD].State = (uint8_t) (PORTA & (C_Pressed | D_Pressed));
        if (Keys[CD].State == (C_Pressed | D_Pressed)) {
            SkipHandling = 5;
            if (Keys[CD].Tick100ms < 15) {
                DevMode &= ~(UNBIND_ACTIVE | MODE_CHANGE_ACTIVE);
                DevMode |= BIND_ACTIVE;
                tick3_100ms = 100;
            } else if ((Keys[CD].Tick100ms >= 35) && (Keys[CD].Tick100ms < 49)) {
                DevMode &= ~BIND_ACTIVE;
            } else if ((Keys[CD].Tick100ms >= 50) && (Keys[CD].Tick100ms < 85)) {
                DevMode &= ~(BIND_ACTIVE | MODE_CHANGE_ACTIVE);
                DevMode |= UNBIND_ACTIVE;
                tick3_100ms = 100;
            } else if ((Keys[CD].Tick100ms >= 85) && (Keys[CD].Tick100ms < 99)) {
                DevMode &= ~UNBIND_ACTIVE;
            } else if (Keys[CD].Tick100ms >= 100) {
                DevMode &= ~(BIND_ACTIVE | UNBIND_ACTIVE);
                DevMode |= MODE_CHANGE_ACTIVE;
                tick3_100ms = 100;
            }
            Keys[CD].Tick100ms++;

            Keys[C].StateTemp = 0;
            Keys[D].StateTemp = 0;
        } else {
            Keys[CD].Tick100ms = 0;
        }

        if (tick3_100ms != 0) {
            tick3_100ms--;
            switch (DevMode & 0x07) {
                case BIND_ACTIVE:
                    LED = ON;
                    break;
                case UNBIND_ACTIVE:
                    if (LedPulseTick_100ms++ > 2) {
                        LED = ~LED;
                        LedPulseTick_100ms = 0;
                    }
                    break;
                case MODE_CHANGE_ACTIVE:
                    if (LedPulseTick_100ms++ > 0) {
                        LED = ~LED;
                        LedPulseTick_100ms = 0;
                    }
                    break;
                default:
                    LED = OFF;
                    break;
            }
        } else {
            DevMode &= ~(BIND_ACTIVE | UNBIND_ACTIVE | MODE_CHANGE_ACTIVE);
            LED = OFF;
        }

        if (SkipHandling != 0) {
            if (Keys[CD].State == 0) {
                SkipHandling--;
            }
        } else {
            if ((DevMode & BIND_ACTIVE) != 0) { //Режим привязки---------------------------------
                for (uint8_t chn = 0; chn < 4; chn++) {
                    if (Keys[chn].State != Keys[chn].StateTemp) {
                        if (Keys[chn].State == 0) {
                            noolite_send(chn, CMD_Bind, 0, &noo_send_data[0]);
                            DevMode &= ~BIND_ACTIVE;
                        }
                        Keys[chn].StateTemp = Keys[chn].State;
                    }
                }
            } else if ((DevMode & UNBIND_ACTIVE) != 0) {
                for (uint8_t chn = 0; chn < 4; chn++) {
                    if (Keys[chn].State != Keys[chn].StateTemp) {
                        if (Keys[chn].State == 0) {
                            noolite_send(chn, CMD_Unbind, 0, &noo_send_data[0]);
                            DevMode &= ~UNBIND_ACTIVE;
                            LED = OFF;
                        }
                        Keys[chn].StateTemp = Keys[chn].State;
                    }
                }
            } else if ((DevMode & MODE_CHANGE_ACTIVE) != 0) {
                for (uint8_t chn = 0; chn < 4; chn++) {
                    if (Keys[chn].State != Keys[chn].StateTemp) {
                        if (Keys[chn].State == 0) {
                            CLRWDT();
                            LED = ON;
                            FlashEraseRow(TYPE_ADR);
                            CLRWDT();
                            FlashWrite(TYPE_ADR, chn);
                            CLRWDT();
                            FlashWrite(TYPE_ADR + 1, 0x5A);
                            CLRWDT();
                            __delay_ms(500);
                            CLRWDT();
                            DevType = Init_TypeFromFlash(&Type[0]);
                            DevMode &= ~MODE_CHANGE_ACTIVE;
                            LED = OFF;
                        }
                        Keys[chn].StateTemp = Keys[chn].State;
                    }
                }
            } else { //нормальный режим---------------------------------------------
                //KEYS ACTION----------
                for (uint8_t chn = 0; chn < 4; chn++) {
                    if (Keys[chn].State != Keys[chn].StateTemp) {
                        switch (DevType) {
                            case 0:
                                KeyOffHandler(&Keys[chn], chn, CMD_Switch, &noo_send_data[0]);
                                break;
                            case 1:
                                if ((chn == 0) || (chn == 2)) {
                                    KeyOffHandler(&Keys[chn], chn, CMD_OFF, &noo_send_data[0]);
                                } else {
                                    KeyOffHandler(&Keys[chn], chn, CMD_ON, &noo_send_data[0]);
                                }
                                break;
                            case 2:
                                if (chn > 1) {
                                    if (Keys[chn].State == 0) {
                                        if (Keys[chn].Tick100ms < 10) { //key pressed SHORT
                                            if (chn > 1) {
                                                noolite_send(chn, CMD_Load_Preset, 0, &noo_send_data[0]);
                                            }
                                        }
                                    }
                                } else {
                                    KeyOffHandler(&Keys[chn], chn, CMD_Switch, &noo_send_data[0]);
                                }
                                break;
                            case 3:
                                if (chn > 1) { //pulse mode for C and D channels
                                    if (Keys[chn].State) {
                                        noolite_send(chn, CMD_ON, 0, &noo_send_data[0]);
                                    } else {
                                        noolite_send(chn, CMD_OFF, 0, &noo_send_data[0]);
                                    }
                                } else {
                                    KeyOffHandler(&Keys[chn], chn, CMD_Switch, &noo_send_data[0]);
                                }
                                break;
                        }
                        OffDelayTicks_100ms = 3;
                        Keys[chn].StateTemp = Keys[chn].State;
                    }
                    //передача начальной команды при длительном удержании
                    switch (DevType) {
                        case 0:
                            KeyLongHandler(&Keys[chn], chn, CMD_Bright_Back, 9, &noo_send_data[0]);
                            break;
                        case 1:
                            if (chn == 0 || chn == 2) {
                                KeyLongHandler(&Keys[chn], chn, CMD_Bright_Down, 9, &noo_send_data[0]);
                            } else {
                                KeyLongHandler(&Keys[chn], chn, CMD_Bright_Up, 9, &noo_send_data[0]);
                            }
                            break;
                        case 2:
                            if (chn < 2) {
                                KeyLongHandler(&Keys[chn], chn, CMD_Bright_Back, 9, &noo_send_data[0]);
                            } else {
                                KeyLongHandler(&Keys[chn], chn, CMD_Save_Preset, 49, &noo_send_data[0]);
                            }
                            break;
                        case 3:
                            if (chn < 2) {
                                KeyLongHandler(&Keys[chn], chn, CMD_Bright_Back, 9, &noo_send_data[0]);
                            }
                            break;
                    }

                }
            }
        }
        if (OffDelayTicks_100ms != 0) {
            OffDelayTicks_100ms--;
        } else {
            DevMode |= GO_OFF;
        }
        //переход в Sleep на ~100мс
        if (((DevMode & 0x07) == 0) && ((PORTA & All_Pressed) == 0) && ((DevMode & GO_OFF) != 0)) {
            for (uint8_t cellNum = 0; cellNum < 8; cellNum++) {
                uint16_t adrToWrite = (TX_STATUS_ADR + (cellNum * 2));
                if (TxStatus[cellNum] == 0xFFFF) {
                    FlashWrite(adrToWrite, tx_status & 0x02);
                    FlashWrite((adrToWrite + 1), 0x5A);
                    NOP();
                    CLRWDT();
                    break;
                } else {
                    if (cellNum == 7) {
                        FlashEraseRow(TX_STATUS_ADR);
                        FlashWrite(TX_STATUS_ADR, tx_status & 0x02);
                        FlashWrite(TX_STATUS_ADR + 1, 0x5A);
                    }
                }
            }
            VddLatch = 0;
            __delay_ms(15);
        } else {
            WDTCONbits.WDTPS = 0b00110; //1:2048 (Interval 64 ms nominal)
            NOP();
            SLEEP();
            NOP();
            WDTCONbits.WDTPS = 0b00101; //1:1024 (Interval 32 ms nominal)
            NOP();
            SLEEP();
            NOP();
            WDTCONbits.WDTPS = 0b01011; //1:65536 (Interval 2s nominal) (Reset value)
            NOP();
            CLRWDT();
        }

    }
}
