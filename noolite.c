#include <pic16lf1503.h>
#include <stdint.h>
#include <xc.h>


#define _XTAL_FREQ 8000000
/*
 * 2. ����������� ������:
 * 2.3 ��� �������� ���������� 2 ������� ��������: 1 � ����������� - ��� �������, 2 ��� ���������� - ������ ��������
 *  �������� ������ ��������� ��� ���� 3 �������.
 * 2.4 ��� ������� ������� ��� � ��� ���������� �������  CMD_Battery_Low - ��� ������� ���������� � ����������,
 * ��� ��������������� ������ ������� CMD_Battery_Low �� ����������, �.�. ���������� � ������� ������� ���� � ������
 * 2.5 ��� ������� ������� ���������� ��������� �����������:
 *  3 �������� ������� ������������� 40 ms/� ���������� - 200ms / �������� �������� ������ ������� - 8 ���
 * 2.6 ���������� �������� ������� � ����� ������� - 2.
 */


const unsigned char __attribute__((address(0x7EE))) system_settings[] = {'C', 4, '_', 'A', 0x1F, 0xFB, '_', 'A', 0x78, 0x56, 0x34, 0x12, '_', 'B', 0x10, 0, '_'};

unsigned char noo_address_type; //��� ������ 0 - 16 ������, !=0 - 32 ������
unsigned char tx_status; //���������� ������� ��� ��������
unsigned long startaddress; //��������� �����

void RF_Init() {
    //��������� ������
    startaddress = 0;
    startaddress |= ((unsigned int) system_settings[5]) << 8;
    startaddress |= system_settings[4];
    if ((startaddress != 0) && (startaddress != 0xFFFF)) {//16 ������ �����
        noo_address_type = 0;
    } else {//32 ������ �����
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

//�������
#define CMD_OFF                  0 ///������������
#define CMD_Bright_Down          1 
#define CMD_On                   2 ///������������
#define CMD_Bright_Up            3 
#define CMD_Switch               4 
#define CMD_Bright_Back          5 
#define CMD_Set_Brightness       6 
#define CMD_Load_Preset          7 
#define CMD_Save_Preset          8 
#define CMD_Unbind               9 ///������������
#define CMD_Stop_Reg            10 
#define CMD_Bright_Step_Down    11
#define CMD_Bright_Step_Up      12
#define CMD_Bright_Reg          13 
#define CMD_Reserved            14
#define CMD_Bind                15 ///������������, ���������� 8 ��� - ��� ������� 1 - �����������/2 - ���������
#define CMD_Roll_Color          16 
#define CMD_Switch_Color        17 
#define CMD_Switch_Mode         18 
#define CMD_Speed_Mode_Back     19
#define CMD_Battery_Low         20 //������������, ���������� 4 ���� � �����
#define CMD_Send_Temp_Humi      21 //�������� �����������/��������� - ���������� 32 ����, 3-�� ������������, 2 - ���������,  1.0 - �����������/��������� �������
#define CMD_Test_Result         22 //�������� �������� ����������
#define CMD_Temporary_On        25 //�������� ���� �� �������� �����. ����� � 5-��������� ������ ���������� � ���������� 


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
volatile unsigned char noo_send_format; //������
volatile unsigned char noo_send_command; //�������
volatile unsigned char noo_send_data0; //������ 0
volatile unsigned char noo_send_data1; //������ 1
volatile unsigned char noo_send_data2; //������ 2
volatile unsigned char noo_send_data3; //������ 3 - ��������� ���������� ��� ������ ���������
 */


//������� ��� ���������� �����
/*
0-databit     //������������ ���
1-toglbit     //toglbit
2-centrbit    //��� �����
3-goodsend    //���������� ��������
4-tmpcrc      //��������� ��� ��� CRC
5-send_pre    //�������� ���������
6-active_send //����� �� �������� ��������, �������� ��������
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

void noolite_send(uint8_t chn, uint8_t noo_send_command, uint8_t noo_send_format, uint8_t* data) {//�������� ������� ��� ��������-�������������� �������� 32 ������� ������
    unsigned char noo_send_buffer[12]; //����� ��� �������� ������ �� RF
    unsigned char temp_send; //������������� ���������� ��� ��������
    unsigned char countsend; //������� ���������� ���������� ���
    unsigned char countbit; //���������� ������������ ���
    unsigned char startpos; // ������� ������� ����
    unsigned char endpos; // ������� ���������� ����=endpos*8
    SEND_CLRWDT;

    //OSCCON = 0b01101010; //4 ���

    toglbit_inv; //�������������� ���� togl

    //�������� ���� ������� �� �������� (��� ����� �������)
    if (noo_send_command < 16) {//4 ������ �������
        startpos = 10; //���, � �������� �������� ����������
        noo_send_buffer[1] = (noo_send_command << 4) | (1 << 2); //��������� ��� � �������
        if (toglbit) {//��� togl
            setbit(noo_send_buffer[1], 3);
        } else {
            clearbit(noo_send_buffer[1], 3);
        }
    } else {//8 ������ �������
        startpos = 6; //���, � �������� �������� ����������
        noo_send_buffer[0] = (1 << 6); //��������� ���
        if (toglbit) {//��� togl
            setbit(noo_send_buffer[0], 7);
        } else {
            clearbit(noo_send_buffer[0], 7);
        };
        noo_send_buffer[1] = noo_send_command; //����� 8 ������ �������
        noo_send_format = noo_send_format | 4;
    }
    endpos = 2; //������ ��������� ������
    temp_send = noo_send_format & 3;
    if (temp_send == 1) {//1 ���� ������
        //noo_send_buffer[2] = noo_send_data0;
        noo_send_buffer[2] = data[0];
        endpos++;
    } else if (temp_send == 3) {//4 ����� ������
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
    if (noo_address_type != 0) {//32 ������ �����
        noo_send_format = noo_send_format | 0x10;
        noo_send_buffer[endpos] = (unsigned char) (startaddress + chn);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 8);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 16);
        endpos++;
        noo_send_buffer[endpos] = (unsigned char) ((startaddress + chn) >> 24);
        endpos++;
    } else {//16 ������ �����
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
    noo_send_buffer[endpos] = temp_send; //������ CRC � �����
    endpos++;
    SEND_TIMER1000;
    temp_send = 3; //���������� ������� ������
    centrbit_0; //����� ����� ������
    goodsend_0; //����� ����� ���������� ��������
    databit_1; //� ��������� ��� 1
    send_pre_1; //�������� ���������
    countsend = 0; //�������� ���������� � 0
    countbit = 38; //����� ��������� = 37 ��� + 1
    SEND_CLRWDT;
    LED_ON; //������� ����������� (Ton=15��)
    __delay_ms(15); //
    LED_OFF; //
    SEND_TIMER_REG = SHORT1000;
    SEND_TIMER_IF = 0;
    SEND_TIMER_IE = 0;
    while (notgoodsend) {//������������ �� ����� ��������
        if (SEND_TIMER_IF) {//���� �������� ������, ���������� ��������� �����������/������ �����
            NOP();
            if (countsend < countbit) {//�������� ������
                if (notcentrbit) {//���� �� ������ � �����, �� �������� ������ ��������
                    if (notdatabit) {//���� 0
                        TX_MOD_ON;
                    } else {// ���� 1
                        TX_MOD_OFF;
                    }
                    SEND_TIMER_REG = SHORT1000; //����� 0,5 ��
                    centrbit_1;
                } else { //���� ������ � �����, �� �������� ������ �������� ���������������
                    TX_MOD_INV;
                    SEND_TIMER_REG = SHORT1000; //����� 0,5 ��
                    centrbit_0;
                    countsend++;
                }
            } else {//��� ��������, ���� ����� � 1,5 ��, ����� ����� �������� ��� ������� �� �����
                TX_MOD_OFF;
                SEND_TIMER1000;
                SEND_TIMER_REG = LONG1000;
                SEND_CLRWDT;
                send_pre_0; //����� ���� �������� ���������
                countsend = startpos;
                countbit = endpos * 8;
                if (temp_send != 0) {//�������� ������
                    temp_send--;
                } else {//�������� ���������
                    SEND_TIMER_REG = 0; //������������ �����
                    goodsend_1; //�������� ���������
                    break; //����� �� ����� while ��������
                }
            }
            if (notsend_pre) {//�������� ������
                if (testbit(noo_send_buffer[(countsend / 8)], (countsend % 8))) {//���� 1
                    databit_1;
                } else {//���� 0
                    databit_0;
                }
            }
            SEND_TIMER_IF = 0; //����� ����� �������
        }
    } //�������� ��������� ��������
    SEND_TIMER_IF = 0;
    TX_MOD_OFF;
    //LED_OFF;
}









