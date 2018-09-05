#include <pic16lf1503.h>
#include <stdint.h>
#include <xc.h>


#define _XTAL_FREQ 8000000
/*
 * 2. Особенности работы:
 * 2.3 При привязке передаются 2 команды привязки: 1 с расширением - тип датчика, 2 без расширения - просто привязка
 *  перадача команд одинакова для всех 3 режимов.
 * 2.4 При разряде батареи раз в час передается команда  CMD_Battery_Low - для режимов термостата и гигростата,
 * для информационного режима команда CMD_Battery_Low не передается, т.к. информация о разряде батареи есть в данных
 * 2.5 При разряде батареи начинается индикация светодиодом:
 *  3 короткие вспышки длительностью 40 ms/с интервалом - 200ms / интервал повторов группы вспышек - 8 сек
 * 2.6 Количество повторов команды в одной посылке - 2.
 */


const unsigned char __attribute__((address(0x7EE))) system_settings[] = {'C', 4, '_', 'A', 0x1F, 0xFB, '_', 'A', 0x78, 0x56, 0x34, 0x12, '_', 'B', 0x10, 0, '_'};

unsigned char noo_address_type; //тип адреса 0 - 16 битный, !=0 - 32 битный
unsigned char tx_status; //переменная статуса для передачи
unsigned long startaddress; //стартовый адрес

void RF_Init() {
    //получение адреса
    startaddress = 0;
    startaddress |= ((unsigned int) system_settings[5]) << 8;
    startaddress |= system_settings[4];
    if ((startaddress != 0) && (startaddress != 0xFFFF)) {//16 битный адрес
        noo_address_type = 0;
    } else {//32 битный адрес
        startaddress = 0;
        startaddress |= ((unsigned long) system_settings[11]) << 24;
        startaddress |= ((unsigned long) system_settings[10]) << 16;
        startaddress |= ((unsigned long) system_settings[9]) << 8;
        startaddress |= system_settings[8];
        noo_address_type = 1;
    }
    CLRWDT();
}


#define setbit(reg, bit)         reg |= (1<<bit)
#define clearbit(reg, bit)       reg &= (~(1<<bit))
#define invbit(reg, bit)         reg ^= (1<<bit)
#define testbit(reg, bit)        ((reg & (1<<bit)) != 0)
#define nottestbit(reg, bit)     ((reg & (1<<bit)) == 0)
#define setdata(reg, datareg, mask)   reg ^=((reg^datareg)& mask)

//команды
#define CMD_OFF                  0 ///используется
#define CMD_Bright_Down          1 
#define CMD_On                   2 ///используется
#define CMD_Bright_Up            3 
#define CMD_Switch               4 
#define CMD_Bright_Back          5 
#define CMD_Set_Brightness       6 
#define CMD_Load_Preset          7 
#define CMD_Save_Preset          8 
#define CMD_Unbind               9 ///используется
#define CMD_Stop_Reg            10 
#define CMD_Bright_Step_Down    11
#define CMD_Bright_Step_Up      12
#define CMD_Bright_Reg          13 
#define CMD_Reserved            14
#define CMD_Bind                15 ///используется, расширение 8 бит - тип датчика 1 - температура/2 - влажность
#define CMD_Roll_Color          16 
#define CMD_Switch_Color        17 
#define CMD_Switch_Mode         18 
#define CMD_Speed_Mode_Back     19
#define CMD_Battery_Low         20 //используется, передается 4 раза в сутки
#define CMD_Send_Temp_Humi      21 //Передача температуры/влажности - расширение 32 бита, 3-не используется, 2 - влажность,  1.0 - температура/параметры датчика
#define CMD_Test_Result         22 //Передача тестовой информации
#define CMD_Temporary_On        25 //Включить свет на заданное время. Время в 5-секундных тактах передается в расширении 


#define   TX_MOD_ON        LATAbits.LATA5 = 1
#define   TX_MOD_OFF       LATAbits.LATA5 = 0 
#define   TX_MOD_INV       invbit(LATA,5)


#define   LED_ON           LATCbits.LATC3 = 1
#define   LED_OFF          LATCbits.LATC3 = 0
#define   LED_INV          invbit(LATC,3)


/*
// 4MHz
#define   SHORT1000  196 //good
#define   LONG1000   67  //good
#define   SEND_TIMER1000  OPTION_REG &= 0b11000000;  OPTION_REG |= 0b00000010

#define   SEND_TIMER_REG  TMR0
#define   SEND_TIMER_IF   TMR0IF
#define   SEND_TIMER_IE   TMR0IF
#define   SEND_CLRWDT     CLRWDT() 
 */


// 1MHz
#define   SHORT1000  196 //good
#define   LONG1000   67  //good
// 1MHz
//#define   SEND_TIMER1000  OPTION_REG &= 0b11000000
// 4MHz
//#define   SEND_TIMER1000  OPTION_REG &= 0b11000000; OPTION_REG |= 0b00000010
// 8MHz
#define   SEND_TIMER1000  OPTION_REG &= 0b11000000; OPTION_REG |= 0b00000011
// 16MHz
//#define   SEND_TIMER1000  OPTION_REG &= 0b11000000; OPTION_REG |= 0b00000100

#define   SEND_TIMER_REG  TMR0
#define   SEND_TIMER_IF   TMR0IF
#define   SEND_TIMER_IE   TMR0IE
#define   SEND_CLRWDT     CLRWDT()

/*
volatile unsigned char noo_send_format; //формат
volatile unsigned char noo_send_command; //команда
volatile unsigned char noo_send_data0; //данные 0
volatile unsigned char noo_send_data1; //данные 1
volatile unsigned char noo_send_data2; //данные 2
volatile unsigned char noo_send_data3; //данные 3 - временная переменная для порога влажности
 */


//макросы для статусного байта
/*
0-databit     //передаваемый бит
1-toglbit     //toglbit
2-centrbit    //бит цетра
3-goodsend    //завершение передачи
4-tmpcrc      //временный бит для CRC
5-send_pre    //передача преамбулы
6-active_send //буфер на передачу заполнен, передача запущена
 */

#define   notactive_send   nottestbit(tx_status,6)
#define   active_send	   testbit(tx_status,6)
#define   active_send_1    setbit(tx_status,6)
#define   active_send_0    clearbit(tx_status,6)

#define   notsend_pre     nottestbit(tx_status,5)
#define   send_pre	  testbit(tx_status,5)
#define   send_pre_1      setbit(tx_status,5)
#define   send_pre_0      clearbit(tx_status,5)

#define   nottmpCRC     nottestbit(tx_status,4)
#define   tmpCRC	testbit(tx_status,4)
#define   tmpCRC_1      setbit(tx_status,4)
#define   tmpCRC_0      clearbit(tx_status,4)

#define   notgoodsend   nottestbit(tx_status,3)
#define   goodsend	testbit(tx_status,3)
#define   goodsend_1    setbit(tx_status,3)
#define   goodsend_0    clearbit(tx_status,3)

#define   centrbit      testbit(tx_status,2)
#define   notcentrbit   nottestbit(tx_status,2)
#define   centrbit_1    setbit(tx_status,2)
#define   centrbit_0    clearbit(tx_status,2)

#define   toglbit       testbit(tx_status,1)
#define   toglbit_1     setbit(tx_status,1)
#define   toglbit_0     clearbit(tx_status,1)
#define   toglbit_inv   invbit(tx_status,1)

#define   databit       testbit(tx_status,0)
#define   notdatabit    nottestbit(tx_status,0)
#define   databit_1     setbit(tx_status,0)
#define   databit_0     clearbit(tx_status,0)

void noolite_send(uint8_t chn, uint8_t noo_send_command, uint8_t noo_send_format, uint8_t* data) {//отправка команды для передачи-протестировать передачу 32 битного адреса
    unsigned char noo_send_buffer[12]; //буфер для передачи данных на RF
    unsigned char temp_send; //универсальная переменная для передачи
    unsigned char countsend; //счетчик количества переданных бит
    unsigned char countbit; //количество передаваемых бит
    unsigned char startpos; // позиция первого бита
    unsigned char endpos; // позиция последнего бита=endpos*8
    SEND_CLRWDT;

    //OSCCON = 0b01101010; //4 МГц

    toglbit_inv; //инвертирование бита togl

    //проверка типа команды по значению (без учета формата)
    if (noo_send_command < 16) {//4 битные команды
        startpos = 10; //бит, с которого начинать передавать
        noo_send_buffer[1] = (noo_send_command << 4) | (1 << 2); //стартовый бит и команда
        if (toglbit) {//бит togl
            setbit(noo_send_buffer[1], 3);
        } else {
            clearbit(noo_send_buffer[1], 3);
        }
    } else {//8 битные команды
        startpos = 6; //бит, с которого начинать передавать
        noo_send_buffer[0] = (1 << 6); //стартовый бит
        if (toglbit) {//бит togl
            setbit(noo_send_buffer[0], 7);
        } else {
            clearbit(noo_send_buffer[0], 7);
        };
        noo_send_buffer[1] = noo_send_command; //целая 8 битная команда
        noo_send_format = noo_send_format | 4;
    }
    endpos = 2; //начало остальных данных
    temp_send = noo_send_format & 3;
    if (temp_send == 1) {//1 байт данных
        //noo_send_buffer[2] = noo_send_data0;
        noo_send_buffer[2] = data[0];
        endpos++;
    } else if (temp_send == 3) {//4 байта данных
        //noo_send_buffer[2] = noo_send_data0;
        //noo_send_buffer[3] = noo_send_data1;
        //noo_send_buffer[4] = noo_send_data2;
        //noo_send_buffer[5] = noo_send_data3;
        noo_send_buffer[2] = data[0];
        noo_send_buffer[3] = data[1];
        noo_send_buffer[4] = data[2];
        noo_send_buffer[5] = data[3];
        endpos += 4;
    }
    if (noo_address_type != 0) {//32 битный адрес
        noo_send_format = noo_send_format | 0x10;
        noo_send_buffer[endpos] = (unsigned char) (startaddress + chn);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 8);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 16);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 24);
        endpos++;
    } else {//16 битный адрес
        noo_send_buffer[endpos] = (unsigned char) (startaddress + chn);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 8);
        endpos++;
    }
    noo_send_buffer[endpos] = noo_send_format;
    endpos++;
    temp_send = 0; //crc byte
    countbit = (unsigned char) (endpos * 8);
    countsend = startpos + 1;
    while (countsend < countbit) {
        if (testbit(noo_send_buffer[(countsend / 8)], (countsend % 8))) {
            temp_send = temp_send^0x01;
        }
        tmpCRC_0;
        if (testbit(temp_send, 0)) {
            temp_send = temp_send^0x18;
            tmpCRC_1;
        }
        temp_send = temp_send >> 1;
        clearbit(temp_send, 7);
        if (tmpCRC) {
            setbit(temp_send, 7);
        }
        countsend++;
        SEND_CLRWDT;
    }
    noo_send_buffer[endpos] = temp_send; //запись CRC в буфер
    endpos++;
    SEND_TIMER1000;
    temp_send = 3; //количество передач команд
    centrbit_0; //сброс флага центра
    goodsend_0; //сброс флага завершения передачи
    databit_1; //в преамбуле все 1
    send_pre_1; //передача преамбулы
    countsend = 0; //начинать передавать с 0
    countbit = 38; //длина преамбулы = 37 бит + 1
    SEND_CLRWDT;
    LED_ON; //моргаем светодиодом (Ton=15мс)
    __delay_ms(15); //
    LED_OFF; //
    SEND_TIMER_REG = SHORT1000;
    SEND_TIMER_IF = 0;
    SEND_TIMER_IE = 0;
    while (notgoodsend) {//зацикливание на время передачи
        if (SEND_TIMER_IF) {//если сработал таймер, выполнение процедуры выставления/снятия битов
            NOP();
            if (countsend < countbit) {//передаем данные
                if (notcentrbit) {//если не попали в центр, то передаем первую половину
                    if (notdatabit) {//если 0
                        TX_MOD_ON;
                    } else {// если 1
                        TX_MOD_OFF;
                    }
                    SEND_TIMER_REG = SHORT1000; //пауза 0,5 мс
                    centrbit_1;
                } else { //если попали в центр, то передаем вторую половину инвертированной
                    TX_MOD_INV;
                    SEND_TIMER_REG = SHORT1000; //пауза 0,5 мс
                    centrbit_0;
                    countsend++;
                }
            } else {//все передали, даем паузу в 1,5 мс, затем снова передаем или выходим из цикла
                TX_MOD_OFF;
                SEND_TIMER1000;
                SEND_TIMER_REG = LONG1000;
                SEND_CLRWDT;
                send_pre_0; //снять флаг передачи преамбулы
                countsend = startpos;
                countbit = endpos * 8;
                if (temp_send != 0) {//передача данных
                    temp_send--;
                } else {//передача завершена
                    SEND_TIMER_REG = 0; //максимальная пауза
                    goodsend_1; //передача завершена
                    break; //выход из цикла while передачи
                }
            }
            if (notsend_pre) {//передача данных
                if (testbit(noo_send_buffer[(countsend / 8)], (countsend % 8))) {//если 1
                    databit_1;
                } else {//если 0
                    databit_0;
                }
            }
            SEND_TIMER_IF = 0; //сброс флага таймера
        }
    } //ожидание окончания передачи
    SEND_TIMER_IF = 0;
    TX_MOD_OFF;
    //LED_OFF;
}









