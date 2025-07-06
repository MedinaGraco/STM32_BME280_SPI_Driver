/*
 * BME280.h
 *
 *  Created on: Jun 30, 2025
 *      Author: DELL
 */

#ifndef BME280_SPI_H_
#define BME280_SPI_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>


/*********************************************
 * BME280 Registri
 *********************************************/

// Identifikacija
#define BME280_REG_ID               0xD0  // ID registar
#define BME280_REG_RESET            0xE0  // Soft reset registar

// Status
#define BME280_REG_STATUS           0xF3  // Status registar

// Kontrola
#define BME280_REG_CTRL_HUM         0xF2  // Kontrola vlage
#define BME280_REG_CTRL_MEAS        0xF4  // Kontrola mjerenja (temp + pritisak)
#define BME280_REG_CONFIG           0xF5  // Konfiguracija (filter, standby, SPI)

// Kalibracioni podaci
#define BME280_REG_CALIB00          0x88  // Kalibracioni podaci (od 0x88 do 0xA1)
#define BME280_REG_CALIB26          0xE1  // Kalibracioni podaci (od 0xE1 do 0xF0)

// Mjerni podaci
#define BME280_REG_PRESS_MSB        0xF7
#define BME280_REG_PRESS_LSB        0xF8
#define BME280_REG_PRESS_XLSB       0xF9

#define BME280_REG_TEMP_MSB         0xFA
#define BME280_REG_TEMP_LSB         0xFB
#define BME280_REG_TEMP_XLSB        0xFC

#define BME280_REG_HUM_MSB          0xFD
#define BME280_REG_HUM_LSB          0xFE

/*********************************************
 * Softverski reset kod
 *********************************************/
#define BME280_SOFT_RESET_CMD       0xB6

/*********************************************
 * Status bitovi
 *********************************************/
#define BME280_STATUS_MEASURING     0x08  // 1 = mjerenje u toku
#define BME280_STATUS_IM_UPDATE     0x01  // 1 = update kalibracionih podataka


/*********************************************
 * Struktura koja opisuje senzor
 *********************************************/

typedef struct{

	SPI_HandleTypeDef* spiHandle;
	GPIO_TypeDef* CS_port;
	uint16_t CS_PIN;

	float temperature;
	float humidity;
	float pressure;
    // Kalibracioni parametri
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
}BME280_Device_t;

/*********************************************
 * Low level funkcije
 *********************************************/
HAL_StatusTypeDef BME280_ReadReg(BME280_Device_t* dev, uint8_t address, uint8_t* data);
HAL_StatusTypeDef BME280_WriteReg(BME280_Device_t* dev,uint8_t address, uint8_t data);
HAL_StatusTypeDef BME280_ReadRegs(BME280_Device_t* dev, uint8_t address, uint8_t* buffer, uint8_t length);

/*********************************************
*Funkcija za inicijalizaciju
*********************************************/
HAL_StatusTypeDef BME280_Init(BME280_Device_t* dev, SPI_HandleTypeDef* hspi,GPIO_TypeDef* CS_port, uint16_t CS_pin);

/*********************************************
*Funkcija za citanje podataka
*********************************************/
HAL_StatusTypeDef BME280_ReadCompensatedData(BME280_Device_t* dev);


/* * Ove funkcije se koriste za početak i završetak SPI komunikacije.
 * Potrebno ih je pozivati unutar funkcija za čitanje i pisanje SPI registra.
 *
 * BME280_CS_LOW(dev);   // Aktivira komunikaciju (CS = 0)
 * BME280_CS_HIGH(dev);  // Završava komunikaciju (CS = 1) */
void BME280_CS_LOW(BME280_Device_t* dev);
void BME280_CS_HIGH(BME280_Device_t* dev);

#endif /* BME280_SPI_H_ */
