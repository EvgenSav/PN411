/* 
 * File:   noolite.h
 * Author: Evgen
 *
 * Created on 5 мая 2017 г., 12:05
 */
#include <stdint.h>
void noolite_send(uint8_t chn, uint8_t noo_send_command, uint8_t noo_send_format, uint8_t* data);
void RF_Init();
