/*
 * BMP280.h
 *
 *  Created on: 28. 10. 2017
 *      Author: CyberPunkTECH
 */

#ifndef BMP280_H_
#define BMP280_H_


#define BMP280_dev_address 0xEE
extern signed long temperature_raw, pressure_raw;
extern unsigned short dig_T1, dig_P1;
extern signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
extern float temperature, pressure, altitude, init_height;

extern uint8_t I2C_Read_Register(uint8_t device_adr, uint8_t internal_adr);

extern void I2C_Write_Register(uint8_t device_adr, uint8_t internal_adr, uint8_t data);

extern void BMP280_get_calib_values(void);

extern void BMP280_init(void);

extern void BMP280_calc_values(void);

#endif /* BMP280_H_ */
