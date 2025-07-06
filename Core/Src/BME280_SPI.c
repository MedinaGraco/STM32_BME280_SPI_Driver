/*
 * BME280.c
 *
 *  Created on: Jul 2, 2025
 *      Author: DELL
 */
#include "main.h"
#include "BME280_SPI.h"
#include "math.h"

/*********************************************
 * Low level funkcije
*********************************************/

void BME280_CS_LOW(BME280_Device_t* dev) {
    HAL_GPIO_WritePin(dev->CS_port, dev->CS_PIN, GPIO_PIN_RESET);
}

void BME280_CS_HIGH(BME280_Device_t* dev) {
    HAL_GPIO_WritePin(dev->CS_port, dev->CS_PIN, GPIO_PIN_SET);
}


HAL_StatusTypeDef BME280_ReadReg(BME280_Device_t* dev, uint8_t address, uint8_t* data){

	HAL_StatusTypeDef status;
	uint8_t tx = address | 0x80; /*postavljanje MSB na vrijednost 1*/

	BME280_CS_LOW(dev); /*pokretanje komunikacije*/

	status = HAL_SPI_Transmit(dev->spiHandle,&tx,1,HAL_MAX_DELAY);
	if(status != HAL_OK){
		BME280_CS_HIGH(dev);
		return status;
	}

	status = HAL_SPI_Receive(dev->spiHandle,data,1,HAL_MAX_DELAY);
	BME280_CS_HIGH(dev);

	return status;
}

HAL_StatusTypeDef BME280_WriteReg(BME280_Device_t* dev,uint8_t address, uint8_t data){
	HAL_StatusTypeDef status;
	uint8_t tx[2];
	tx[0] = address & 0x7f; /*adresa s postavljenim MSB na 0*/
	tx[1] = data;

	BME280_CS_LOW(dev);
	status = HAL_SPI_Transmit(dev->spiHandle,tx,2,HAL_MAX_DELAY);
	if(status != HAL_OK){
		BME280_CS_HIGH(dev);
		return status;
	}

	BME280_CS_HIGH(dev);
	return status;
}
HAL_StatusTypeDef BME280_ReadRegs(BME280_Device_t* dev, uint8_t address,uint8_t* buffer, uint8_t length){
	HAL_StatusTypeDef status;
	uint8_t tx = address | 0x80; /*MSB=1*/

	BME280_CS_LOW(dev);

	status = HAL_SPI_Transmit(dev->spiHandle,&tx,1,HAL_MAX_DELAY);
	if(status != HAL_OK){
		BME280_CS_HIGH(dev);
		return status;
	}
	status = HAL_SPI_Receive(dev->spiHandle,buffer,length, HAL_MAX_DELAY);

	BME280_CS_HIGH(dev);
	return status;

}

/*********************************************
 * Funkcija za inicijalizaciju
*********************************************/

HAL_StatusTypeDef BME280_Init(BME280_Device_t* dev, SPI_HandleTypeDef* hspi,GPIO_TypeDef* CS_port, uint16_t CS_pin){

	/*Inicijalizacija strukture*/

	dev ->spiHandle = hspi;
	dev ->CS_port = CS_port;
	dev ->CS_PIN = CS_pin;

	dev ->temperature = 0.0f;
	dev ->humidity = 0.0f;
	dev ->pressure = 0.0f;

	/*Reset senzora*/

	HAL_StatusTypeDef status;
	status = BME280_WriteReg(dev,BME280_REG_RESET, BME280_SOFT_RESET_CMD);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(100);

	/*Provjera ID-a*/
	uint8_t data;
	status = BME280_ReadReg(dev,BME280_REG_ID,&data);
    if (status != HAL_OK) {
        return status;
    }
	if(data != 0x60){
		return HAL_ERROR;
	}

	/*Konfiguracija senzora*/


	status = BME280_WriteReg(dev,BME280_REG_CTRL_HUM,0x01); /*Oversampling x1 za vlažnost*/
    if (status != HAL_OK) {
        return status;
    }

	status = BME280_WriteReg(dev,BME280_REG_CTRL_MEAS,0x27); /*Oversampling x1 za temperaturu, pritisak i postavljanje normalnog moda rada*/
    if (status != HAL_OK) {
        return status;
    }

	status = BME280_WriteReg(dev,BME280_REG_CONFIG, 0x00);/*iskljucen filter*/
    if (status != HAL_OK) {
        return status;
    }
}
/*********************************************
* Funkcija za čitanje kalibracije
* *********************************************/

HAL_StatusTypeDef BME280_ReadCalibration(BME280_Device_t *dev){

    uint8_t calib[26];
    uint8_t calib_h[7];
    HAL_StatusTypeDef status;

    // temperatura i pritisak kalibracija

    status = BME280_ReadRegs(dev, 0x88, calib, 26);
    if (status != HAL_OK)
    	return status;
    //u memoriji se primjenjuje LittleEndian zapis, tako da je potrebno MSB bajt staviti na znacajniju poziciju
    dev->dig_T1 = (uint16_t)(calib[1] << 8)| calib[0];
    dev->dig_T2 = (int16_t)(calib[3] << 8)| calib[2];
    dev->dig_T3 = (int16_t)(calib[5] << 8)| calib[4];
    dev->dig_P1 = (uint16_t)(calib[7] << 8) | calib[6];
    dev->dig_P2 = (int16_t)(calib[9] << 8) | calib[8];
    dev->dig_P3 = (int16_t)(calib[11] << 8) | calib[10];
    dev->dig_P4 = (int16_t)(calib[13] << 8) | calib[12];
    dev->dig_P5 = (int16_t)(calib[15] << 8) | calib[14];
    dev->dig_P6 = (int16_t)(calib[17] << 8) | calib[16];
    dev->dig_P7 = (int16_t)(calib[19] << 8) | calib[18];
    dev->dig_P8 = (int16_t)(calib[21] << 8) | calib[20];
    dev->dig_P9 = (int16_t)(calib[23] << 8) | calib[22];

    dev->dig_H1 = calib[25];
    // citanje podataka o kalibraciji
    status = BME280_ReadRegs(dev, 0xE1, calib_h, 7);
    if (status != HAL_OK) return status;


    dev->dig_H2 = (int16_t)(calib_h[1] << 8) | calib_h[0];
    dev->dig_H3 = calib_h[2];
    dev->dig_H4 = (int16_t)((calib_h[3] << 4) | (calib_h[4] & 0x0F));
    dev->dig_H5 = (int16_t)((calib_h[5] << 4) | (calib_h[4] >> 4));
    dev->dig_H6 = (int8_t)calib_h[6];


    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadCompensatedData(BME280_Device_t* dev){
	uint8_t data[8];
	HAL_StatusTypeDef status;

	status = BME280_ReadRegs(dev, 0xF7, data, 8);
	if (status != HAL_OK) return status;

	int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
	int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
	int32_t adc_H = ((int32_t)data[6] << 8) | data[7];


	//proracun temperature koristeci kalibracijske parametre (formula se nalazi u dokumentaciji)

    int32_t var1, var2, t_fine;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) *
            ((int32_t)dev->dig_T3)) >> 14;
    t_fine = var1 + var2;
    float temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f;

    //proracun pritiska

    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)dev->dig_P6;
    var2_p += ((var1_p * (int64_t)dev->dig_P5) << 17);
    var2_p += ((int64_t)dev->dig_P4) << 35;
    var1_p = (((var1_p * var1_p * (int64_t)dev->dig_P3) >> 8) + ((var1_p * (int64_t)dev->dig_P2) << 12));
    var1_p = (((((int64_t)1) << 47) + var1_p) * (int64_t)dev->dig_P1) >> 33;

    if (var1_p == 0) return HAL_ERROR; // Division by zero protection

    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2_p = ((int64_t)dev->dig_P8 * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dev->dig_P7) << 4);
    float pressure = p / 25600.0f;

    // proracun vlaznosti

    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dev->dig_H4) << 20) -
                    (((int32_t)dev->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * (int32_t)dev->dig_H6) >> 10) *
                       (((v_x1_u32r * (int32_t)dev->dig_H3) >> 11) + ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) * (int32_t)dev->dig_H2 + 8192) >> 14));
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)dev->dig_H1) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    float humidity = (v_x1_u32r >> 12) / 1024.0f;


    dev->temperature = temperature;
    dev->pressure = pressure;
    dev->humidity = humidity;

    return HAL_OK;
}






