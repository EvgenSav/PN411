/*
 * File:   config.c
 * Author: Evgeny
 *
 * Created on 8 сентября 2018 г., 9:34
 */


#include <xc.h>
#include <pic16lf1503.h>
#include "periph_config.h"
#include <stdint.h>


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
    OPTION_REGbits.nWPUEN = 0; //должен быть сброшен для возможности редактирования WPUAn
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
