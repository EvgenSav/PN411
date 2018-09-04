/*
 * File:   main.c
 * Author: Evgeny
 *
 * Created on 31 августа 2017 г., 12:27
 */
//команды
#define CMD_OFF                  0 ///используетс€
#define CMD_Bright_Down          1 
#define CMD_ON                   2 ///используетс€
#define CMD_Bright_Up            3 
#define CMD_Switch               4 
#define CMD_Bright_Back          5 
#define CMD_Set_Brightness       6 
#define CMD_Load_Preset          7 
#define CMD_Save_Preset          8 
#define CMD_Unbind               9 ///используетс€
#define CMD_Stop_Reg            10 
#define CMD_Bright_Step_Down    11
#define CMD_Bright_Step_Up      12
#define CMD_Bright_Reg          13 
#define CMD_Reserved            14
#define CMD_Bind                15 ///используетс€, расширение 8 бит - тип датчика 1 - температура/2 - влажность
#define CMD_Roll_Color          16 
#define CMD_Switch_Color        17 
#define CMD_Switch_Mode         18 
#define CMD_Speed_Mode_Back     19
#define CMD_Battery_Low         20 //используетс€, передаетс€ 4 раза в сутки
#define CMD_Send_Temp_Humi      21 //ѕередача температуры/влажности - расширение 32 бита, 3-не используетс€, 2 - влажность,  1.0 - температура/параметры датчика
#define CMD_Test_Result         22 //ѕередача тестовой информации
#define CMD_Temporary_On        25 //¬ключить свет на заданное врем€. ¬рем€ в 5-секундных тактах передаетс€ в расширении 

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
#include "noolite.h"
#include "FLASH.h"
#include <stdint.h>
#define _XTAL_FREQ 8000000


//CONFIG-------------------------------------------------------------------------------------------------------------------
#define  AlarmBatteryVoltage 2.5    //значение напр€жени€ батареи ниже которого происходит сигнализаци€
const uint16_t AlarmBattVoltage = (unsigned int) (1.024 / AlarmBatteryVoltage * 1023); //419
//--------------------------------------------------------------------------------------------------------------------------


#define LED         LATCbits.LATC3
#define RF          LATAbits.LATA5
#define VddLatch    LATCbits.LATC5

enum {
    OFF = 0,
    ON = 1
};


//modes of operation

//–адиоканал--------------------------------------------------------------------------------
extern unsigned char tx_status; //переменна€ статуса дл€ передачи
unsigned char noo_send_data[4] = {0, 0, 0, 0}; //массив с передаваемыми данными по – 

uint8_t BattLowSent = 0; //команда разр€да батареи отправлена
uint8_t BattLow = 0;

uint8_t LedPulseTick_100ms = 0;

void Init_CLK() {
    //OSCCONbits.IRCF = 0b1101;  //4 MHz 
    //OSCCONbits.IRCF = 0b1011; //1 MHz 
    OSCCONbits.IRCF = 0b1110; //8MHz
    //OSCCONbits.IRCF = 0b1111;// 16MHz
    //0b1111 = 16MHz
    //0b1110 = 8MHz
    //0b1101 = 4MHz
    //0b1100 = 2MHz
    //0b1011 = 1MHz
    //0b1010 = 500 kHz(1)
    //0b1001 = 250 kHz(1)
    //0b1000 = 125 kHz(1)
    //0b0111 = 500 kHz (default upon Reset)
    //0b0110 = 250 kHz
    //0b0101 = 125 kHz
    //0b0100 = 62.5 kHz
    //0b001x = 31.25 kHz
    //0b000x = 31 kHz (LFINTOSC)
    WDTCONbits.WDTPS = 0b01011; //WDT period 2   s
    //0b01010;                  //WDT period 1   s
    //0b01101;                  //WDT period 8   s
    //0b01101;                  //WDT period 8   s
    //0b01110;                  //WDT perios 16  s 
    //0b10000;                  //WDT perios 64  s               
    //0b10010;                  //WDT perios 256 s 
    WDTCONbits.SWDTEN = 1; //WDT ON at startup
    while (!OSCSTATbits.HFIOFR) {
    } //wait for STABLE CPU clock
}

void Init_IO() {
    OPTION_REGbits.nWPUEN = 0; //должен быть сброшен дл€ возможности редактировани€ WPUAn
    TRISA = 0x1F; //PORTA: RA0, RA1, RA2, RA4 as INPUT (RA3 - inout only)
    ANSELA = 0; //PORTA as dig.I/O
    WPUA = 0x08;

    TRISC = 0x00; //PORTC as OUTPUT
    ANSELC = 0; //PORTC as dig.I/O
    LATC = 0x00;
}

void Init_ADC() {
    FVRCON = 0;
    //ADCON0bits.CHS = 0b11101;     //ADC channel is temp. sensor
    ADCON1bits.ADFM = 1; //0;            //левое выравнивание(читаем ADRESH)
    ADCON1bits.ADCS = 0b001; //000;
    //ADCS=0b000;    //Fcy = Fosc/2 = 2us    
    //ADCS=0b110;    //Fcy = Fosc/64
    ADCON1bits.ADPREF = 0b00;

    ADCON0bits.ADON = 0; //disable ADC
}



//ADC channels
#define Battery         31

unsigned int GetAdcValue(uint8_t channel) {
    ADCON0bits.CHS = (uint8_t) (channel & 0x1F); //установка канала ANx дл€ измерени€ 

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


uint8_t mode = 0;

uint8_t tick3_100ms = 0;
uint8_t BattLowTransmitCount = 0;

enum {
    A_Pressed = 0x02,
    B_Pressed = 0x01,
    C_Pressed = 0x04,
    D_Pressed = 0x10
};

enum {
    All_Pressed = 0x17
};

enum {
    BIND_ACTIVE = 0x01,
    UNBIND_ACTIVE = 0x02,
    MODE_CHANGE_ACTIVE = 0x04,
    GO_OFF = 0x08
};

struct {
    uint8_t State;
    uint8_t StateTemp;
    uint8_t Tick100ms;
    uint8_t FirstCmdSent;
} typedef KeyState;

const uint16_t Type[] @ 0x7C0 = {
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

uint8_t DevType = 0;
uint8_t DevMode = 0;

KeyState Keys[5];
#define A  0
#define B  1
#define C  2
#define D  3
#define CD 4

uint8_t SkipHandling = 0;

uint8_t Init_TypeFromFlash() {
    if (((Type[0] >> 8) == 0x5A) && ((Type[0] & 0xFF) < 4)) {
        return (Type[0] & 0xFF);
    } else {
        return 0;
    }
}

void Xz(KeyState* key) {

}

uint16_t battery_value = 0;

void main() {
    Init_IO();
    Init_CLK();

    Init_ADC();
    RF_Init();

    VddLatch = 1;
    DevType = Init_TypeFromFlash();

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
            SkipHandling = 10;
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
            if ((DevMode & BIND_ACTIVE) != 0) { //–ежим прив€зки---------------------------------
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
                            noo_send_data[0] = Type[0] >> 8;
                            noo_send_data[1] = Type[0];
                            noo_send_data[2] = DevType;
                            noolite_send(chn, CMD_Test_Result, 7, &noo_send_data[0]);
                            __delay_ms(500);
                            CLRWDT();
                            LED = ON;
                            FlashEraseRow(0x7C0);
                            CLRWDT();
                            __delay_ms(500);
                            CLRWDT();
                            FlashWrite(0x7C0, chn);
                            FlashWrite(0x7C1, 0x5A);
                            CLRWDT();
                            DevType = Init_TypeFromFlash();
                            DevMode &= ~MODE_CHANGE_ACTIVE;
                            LED = OFF;
                            noo_send_data[0] = Type[0] >> 8;
                            noo_send_data[1] = Type[0];
                            noo_send_data[2] = DevType;
                            noolite_send(chn, CMD_Test_Result, 7, &noo_send_data[0]);
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
                                if (Keys[chn].State == 0) {
                                    if (Keys[chn].Tick100ms < 10) { //key pressed SHORT
                                        noolite_send(chn, CMD_Switch, 0, &noo_send_data[0]);
                                    } else { //key pressed LONG
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        __delay_ms(15);
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        __delay_ms(15);
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        Keys[chn].FirstCmdSent = 0;
                                    }
                                }
                                break;
                            case 1:
                                if (Keys[chn].State == 0) {
                                    if (Keys[chn].Tick100ms < 10) { //key pressed SHORT
                                        if ((chn == 0) || (chn == 2)) {
                                            noolite_send(chn, CMD_OFF, 0, &noo_send_data[0]);
                                        } else {
                                            noolite_send(chn, CMD_ON, 0, &noo_send_data[0]);
                                        }
                                    } else { //key pressed LONG
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        __delay_ms(15);
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        __delay_ms(15);
                                        noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                        Keys[chn].FirstCmdSent = 0;
                                    }
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
                                    if (Keys[chn].State == 0) {
                                        if (Keys[chn].Tick100ms < 10) { //key pressed SHORT
                                            noolite_send(chn, CMD_Switch, 0, &noo_send_data[0]);
                                        } else { //key pressed LONG
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            __delay_ms(15);
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            __delay_ms(15);
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            Keys[chn].FirstCmdSent = 0;
                                        }
                                    }
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
                                    if (Keys[chn].State == 0) {
                                        if (Keys[chn].Tick100ms < 10) { //key pressed SHORT
                                            noolite_send(chn, CMD_Switch, 0, &noo_send_data[0]);
                                        } else { //key pressed LONG
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            __delay_ms(15);
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            __delay_ms(15);
                                            noolite_send(chn, CMD_Stop_Reg, 0, &noo_send_data[0]);
                                            Keys[chn].FirstCmdSent = 0;
                                        }
                                    }
                                }
                                break;
                        }
                        Keys[chn].StateTemp = Keys[chn].State;
                    }
                    //передача начальной команды при длительном удержании
                    switch (DevType) {
                        case 0:
                            if (Keys[chn].State != 0) {
                                if (Keys[chn].Tick100ms > 9) { //долгое нажатие
                                    if (Keys[chn].FirstCmdSent == 0) { //передача начальной команды
                                        noolite_send(chn, CMD_Bright_Back, 0, &noo_send_data[0]);
                                        Keys[chn].FirstCmdSent = 1;
                                    }
                                }
                                Keys[chn].Tick100ms++;
                            } else {
                                Keys[chn].Tick100ms = 0;
                            }
                            break;
                        case 1:
                            if (Keys[chn].State != 0) {
                                if (Keys[chn].Tick100ms > 9) { //долгое нажатие
                                    if (Keys[chn].FirstCmdSent == 0) { //передача начальной команды
                                        if (chn == 0 || chn == 2) {
                                            noolite_send(chn, CMD_Bright_Down, 0, &noo_send_data[0]);
                                        } else {
                                            noolite_send(chn, CMD_Bright_Up, 0, &noo_send_data[0]);
                                        }
                                        Keys[chn].FirstCmdSent = 1;
                                    }
                                }
                                Keys[chn].Tick100ms++;
                            } else {
                                Keys[chn].Tick100ms = 0;
                            }
                            break;
                        case 2:
                            if (chn < 2) {
                                if (Keys[chn].State != 0) {
                                    if (Keys[chn].Tick100ms > 9) { //долгое нажатие
                                        if (Keys[chn].FirstCmdSent == 0) { //передача начальной команды
                                            noolite_send(chn, CMD_Bright_Back, 0, &noo_send_data[0]);
                                            Keys[chn].FirstCmdSent = 1;
                                        }
                                    }
                                    Keys[chn].Tick100ms++;
                                } else {
                                    Keys[chn].Tick100ms = 0;
                                }
                            } else {
                                if (Keys[chn].State != 0) {
                                    if (Keys[chn].Tick100ms > 49) { //долгое нажатие
                                        if (Keys[chn].FirstCmdSent == 0) { //передача начальной команды
                                            noolite_send(chn, CMD_Save_Preset, 0, &noo_send_data[0]);
                                            Keys[chn].FirstCmdSent = 1;
                                        }
                                    }
                                    Keys[chn].Tick100ms++;
                                } else {
                                    Keys[chn].Tick100ms = 0;
                                }
                            }
                            break;
                        case 3:
                            if (chn < 2) {
                                if (Keys[chn].State != 0) {
                                    if (Keys[chn].Tick100ms > 9) { //долгое нажатие
                                        if (Keys[chn].FirstCmdSent == 0) { //передача начальной команды
                                            noolite_send(chn, CMD_Bright_Back, 0, &noo_send_data[0]);
                                            Keys[chn].FirstCmdSent = 1;
                                        }
                                    }
                                    Keys[chn].Tick100ms++;
                                } else {
                                    Keys[chn].Tick100ms = 0;
                                }
                            }
                            break;
                    }

                }
            }
        }

        //        if (tick2_100ms++ > 25) {
        //            
        //            tick2_100ms = 0;
        //        }





        //передача команды разр€да батареи------------------
        if (BattLow && !BattLowSent) {
            for (uint8_t chn = 0; chn < 4; chn++) {
                noolite_send(chn, CMD_Battery_Low, 4, &noo_send_data[0]);
                CLRWDT();
                __delay_ms(50);
                CLRWDT();
            }
            BattLowSent = 1;
        }
        //переход в Sleep на ~100мс
        if (((DevMode & 0x07) == 0) && ((PORTA & All_Pressed) == 0)) {
            VddLatch = 0;
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
