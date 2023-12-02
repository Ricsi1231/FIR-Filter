#include "BMI088.h"
#include "bitManipulation.h"

void setCsPin(IMU *imu, uint8_t part, GPIO_PinState state) 
{
    if(part == GYROSCOPE) 
    {
        HAL_GPIO_WritePin(imu->GYRO_CS_PORT, imu->GYRO_CS_PIN, state);
    }
    else 
    {
        HAL_GPIO_WritePin(imu->ACC_CS_PORT, imu->ACC_CS_PIN, state);
    }
}

uint8_t writeRegister(IMU *imu, uint8_t part, uint8_t address, uint8_t data) 
{
    uint8_t dataSize = 2;
    uint8_t bitPlace = 1;
    uint8_t errorState = 0;
    uint8_t txData[2] = {address, data};

    BIT_CLEAR(txData[0], bitPlace);

    if(part == GYROSCOPE) 
    {
        setCsPin(imu, GYROSCOPE, LOW);
        errorState += HAL_SPI_Transmit(imu->SPI, txData, dataSize, HAL_MAX_DELAY);
        setCsPin(imu, GYROSCOPE, HIGH);
    }
    else
    {
        setCsPin(imu, ACCELOMETER, LOW);
        errorState += HAL_SPI_Transmit(imu->SPI, txData, dataSize, HAL_MAX_DELAY);
        setCsPin(imu, ACCELOMETER, HIGH);
    }

    return errorState;
}

uint8_t readRegister(IMU *imu, uint8_t part, uint8_t address) 
{
    uint8_t dataSize = 2;
    uint8_t bitPlace = 1;
    uint8_t errorState = 0;
    uint8_t txData[2] = {address, 0x00};
    uint8_t rxData[2];

    BIT_SET(txData[0], bitPlace);

    if(part == GYROSCOPE) 
    {
        setCsPin(imu, GYROSCOPE, LOW);
        errorState += HAL_SPI_TransmitReceive(imu->SPI, txData, rxData, dataSize, HAL_MAX_DELAY);
        setCsPin(imu, GYROSCOPE, HIGH);

        if(errorState == HAL_OK) 
        {
            return rxData[0];
        }
    }
    else 
    {
        setCsPin(imu, ACCELOMETER, LOW);
        errorState += HAL_SPI_TransmitReceive(imu->SPI, txData, rxData, dataSize, HAL_MAX_DELAY);
        setCsPin(imu, ACCELOMETER, HIGH);

        if(errorState == HAL_OK) 
        {
            return rxData[0];
        }
    }

    return errorState;
}

uint8_t bmi088Init(IMU *imu, SPI_HandleTypeDef *spi, SPI_TypeDef *spi_port, GPIO_TypeDef *acc_cs_port, uint8_t acc_cs_pin, GPIO_TypeDef *gyro_cs_port, uint8_t gyro_cs_pin, GPIO_TypeDef *int1_port, uint8_t int1_pin, IRQn_Type int1_irq_port, GPIO_TypeDef *int2_port, uint8_t int2_pin, IRQn_Type int2_irq_port) 
{
    if((spi != 0 && spi_port != 0) && (acc_cs_pin != 0 && acc_cs_pin != 0) && (gyro_cs_port != 0 && gyro_cs_pin != 0) && (int1_port != 0 && int1_pin != 0) && (int2_port != 0 && int2_pin != 0)) 
    {
        uint8_t bmiStatus = 0;
        uint8_t chipID = 0;

        imu->SPI = spi;

        imu->ACC_CS_PORT = acc_cs_port;
        imu->ACC_CS_PIN = acc_cs_pin;

        imu->GYRO_CS_PORT = gyro_cs_port;
        imu->GYRO_CS_PIN = gyro_cs_pin;

        imu->INT1_PORT = int1_port;
        imu->INT1_PIN = int1_pin;

        imu->INT2_PORT = int2_port;
        imu->INT2_PIN = int2_pin;

        imu->SPI->Instance = spi_port;
        imu->SPI->Init.Mode = SPI_MODE_MASTER;
        imu->SPI->Init.Direction = SPI_DIRECTION_2LINES;
        imu->SPI->Init.DataSize = SPI_DATASIZE_8BIT;
        imu->SPI->Init.CLKPolarity = SPI_POLARITY_LOW;
        imu->SPI->Init.CLKPhase = SPI_PHASE_1EDGE;
        imu->SPI->Init.NSS = SPI_NSS_SOFT;
        imu->SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        imu->SPI->Init.FirstBit = SPI_FIRSTBIT_MSB;
        imu->SPI->Init.TIMode = SPI_TIMODE_DISABLE;
        imu->SPI->Init.CRCPolynomial = SPI_CLC_POLYNOIMAL_SIZE;
        imu->SPI->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

        if(HAL_SPI_Init(imu->SPI) != HAL_OK) 
        {
            return HAL_ERROR;
        }

        GPIO_InitTypeDef GPIO_InitStruct = {0};

        GPIO_InitStruct.Pin = imu->ACC_CS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(imu->ACC_CS_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = imu->GYRO_CS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(imu->GYRO_CS_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = imu->INT1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(imu->INT1_PORT, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(int1_irq_port, 0, 0);
        HAL_NVIC_EnableIRQ(int1_irq_port);

        GPIO_InitStruct.Pin = imu->INT2_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(imu->INT2_PORT, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(int2_irq_port, 0, 0);
        HAL_NVIC_EnableIRQ(int2_irq_port);

        chipID = readRegister(imu, GYROSCOPE, GYRO_CHIP_ID);

        if(chipID == GYRO_CHIP_ID_VALUE) 
        {
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_LPM1, GYRO_PWR_ON_INIT_VALUE);
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_RANGE, GYRO_RANGE_INIT_VALUE);
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_BANDWIDTH, GYRO_BANDWIDTH_INIT_VALUE);
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_INT_CTRL, GYRO_ENABLE_DATA_READY_INTERRUPT_VALUE);
            bmiStatus += writeRegister(imu, GYROSCOPE, INT3_INT4_IO_CONF, GYRO_CONFIGURE_INTERRUPT);
            bmiStatus += writeRegister(imu, GYROSCOPE, INT3_INT4_IO_MAP, GYRO_CONFIGURE_INTERRUPT_DATA_READY);
        }

        chipID = readRegister(imu, ACCELOMETER, ACC_CHIP_ID);

        if(chipID == ACC_CHIP_ID_VALUE) 
        {
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_PWR_CONF, ACC_PWR_ON_INIT_VALUE);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_PWR_CTRL, ACC_ENABLE);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_CONF, ACC_CONF_INIT_VALUE);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_RANGE, ACC_RANGE_INIT_VALUE);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_INT1_IO_CTRL, ACC_ENABLE_DATA_READY_INTERRUPT);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_INT2_IO_CTRL, ACC_ENABLE_DATA_READY_INTERRUPT);
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_INT_MAP_DATA, ACC_CONFIGURE_INTERRUPT);
        }

        return bmiStatus;
    }

    return HAL_ERROR;
}

uint8_t bmi088SleepSensor(IMU *imu) 
{
    uint8_t bmiStatus = 0;

    bmiStatus += writeRegister(imu, ACCELOMETER, ACC_PWR_CTRL, ACC_SUSPEND_VALUE);
	bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_LPM1, GYRO_SUSPEND_VALUE);

    return bmiStatus;
}

uint8_t bmi088WakeupSensor(IMU *imu) 
{
    uint8_t bmiStatus = 0;

    bmiStatus += writeRegister(imu, ACCELOMETER, ACC_PWR_CTRL, ACC_WAKEUP_VALUE);
	bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_LPM1, GYRO_WAKEUP_VALUE);

    return bmiStatus;
}

uint8_t bmi088SetAccelometer(IMU *imu, uint8_t range, uint16_t bandwidth) 
{
    uint8_t rangeValuesLenght = 4;
	uint8_t rangeValues[4] = {3, 6, 12, 24};
    uint8_t rangeHexValues[4] = {0x00, 0x01, 0x02, 0x03};
    
    uint8_t bandWidhtValuesLenght = 8;
    uint16_t bandwidthValues[8] = {12, 25, 50, 100, 200, 400, 800, 1600};
    uint8_t bandwidthHexValues[8] = {0x05, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
    
    uint8_t bmiStatus = 0;
    uint8_t accRangeGoodValue = 0;
    uint8_t accbandwidthGoodValue = 0;
    
    for(uint8_t i = 0; i < rangeValuesLenght; i++)
    {
        if(range == rangeValues[i]) 
        {
            bmiStatus += writeRegister(imu, ACCELOMETER, ACC_RANGE, rangeHexValues[i]);
            accRangeGoodValue = 1;
        }
    }
    
    for(uint8_t i = 0; i < bandWidhtValuesLenght; i++)
    {
    	if(bandwidth == bandwidthValues[i])
    	{
    	    bmiStatus += writeRegister(imu, ACCELOMETER, ACC_CONF, bandwidthHexValues[i]);
    	    accbandwidthGoodValue = 1;
    	}
    }
 
    if(!accRangeGoodValue || !accbandwidthGoodValue) 
    {
    	return HAL_ERROR;
    }

    return bmiStatus;   
}

uint8_t bmi088ReadAccelometer(IMU *imu) 
{
    uint8_t dataSize = 6;
    uint8_t hexValues[6] = {ACC_X_LSB, ACC_X_MSB, ACC_Y_LSB, ACC_Y_MSB, ACC_Z_LSB, ACC_Z_MSB};
    uint8_t dataValues[6];  
    
    for(uint8_t i = 0; i < dataSize; i++) 
    {
        dataValues[i] = readRegister(imu, ACCELOMETER, hexValues[i]);

        if(dataValues[i] == HAL_ERROR)
        {
            return HAL_ERROR;
        }
    }  
    
    imu->x = (int16_t)((dataValues[0] << 8) | dataValues[1]) * imu->accScaleFactor;
    imu->y = (int16_t)((dataValues[2] << 8) | dataValues[3]) * imu->accScaleFactor;
    imu->z = (int16_t)((dataValues[4] << 8) | dataValues[5]) * imu->accScaleFactor;

    return HAL_OK;  
}

uint8_t bmi088ReadAccelometerDMA(IMU *imu) 
{
    uint8_t bmiStatus = 0;
    uint8_t dataSize = 6;
    uint8_t txData[6];
    
    txData[0] = ACC_X_LSB | 0x80;
    
    setCsPin(imu, ACCELOMETER, LOW);
    bmiStatus += HAL_SPI_TransmitReceive_DMA(imu->SPI, txData, imu->accDmaData, dataSize);
    setCsPin(imu, ACCELOMETER, HIGH);
    
    return bmiStatus;
}

uint8_t bmi088ReadAccelometerDMAComplete(IMU *imu) 
{
    imu->x = (int16_t)((imu->accDmaData[0] << 8) | imu->accDmaData[1]) * imu->accScaleFactor;
    imu->y = (int16_t)((imu->accDmaData[2] << 8) | imu->accDmaData[3]) * imu->accScaleFactor;
    imu->z = (int16_t)((imu->accDmaData[4] << 8) | imu->accDmaData[5]) * imu->accScaleFactor;

    return HAL_OK;
}

uint8_t bmi088setGyroscope(IMU *imu, uint8_t range, uint16_t bandwidth) 
{
    uint8_t rangeValuesLenght = 5;
    uint16_t rangeValues[5] = {125, 250, 500, 1000, 2000};
    uint8_t rangeHexValues[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
    
    uint8_t bandWidhtValuesLenght = 8;
    uint16_t bandwidthValues[8] = {100, 200, 100, 200, 400, 1000, 2000, 2000};
    uint8_t bandwidthHexValues[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    
    uint8_t bmiStatus = 0;
    uint8_t gyroRangeGoodValue = 0;
    uint8_t gyrobandwidthGoodValue = 0;
    
    for(int i = 0; i < rangeValuesLenght; i++)
    {
        if(range == rangeValues[i]) 
        {
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_RANGE, rangeHexValues[i]);
            gyroRangeGoodValue = 1;
            break;
        }
    }
    
    for(int i = 0; i < bandWidhtValuesLenght; i++)
    {
        if(bandwidth == bandwidthValues[i]) 
        {
            bmiStatus += writeRegister(imu, GYROSCOPE, GYRO_BANDWIDTH, bandwidthHexValues[i]);
            gyrobandwidthGoodValue = 1;
            break;
        }
    }
    
    if(!gyroRangeGoodValue || !gyrobandwidthGoodValue) 
    {
    	return HAL_ERROR;
    }

    return bmiStatus;
}

uint8_t bmi088readGyroscope(IMU *imu) 
{
    uint8_t dataSize = 6;
    uint8_t hexValues[6] = {GYRO_X_LSB, GYRO_X_MSB, GYRO_Y_LSB, GYRO_Y_MSB, GYRO_Z_LSB, GYRO_Z_MSB};
    uint8_t dataValues[6];
    
    for(uint8_t i = 0; i < dataSize; i++) 
    {
        dataValues[i] = readRegister(imu, GYROSCOPE, hexValues[i]);

        if(dataValues[i] == HAL_ERROR) 
        {
            return HAL_ERROR;
        }
    }
    imu->x = (int16_t)((dataValues[0] << 8) | dataValues[1]) * imu->gyroScaleFactor;
    imu->y = (int16_t)((dataValues[2] << 8) | dataValues[3]) * imu->gyroScaleFactor;
    imu->z = (int16_t)((dataValues[4] << 8) | dataValues[5]) * imu->gyroScaleFactor;

    return HAL_OK;
}

uint8_t bmi088readGyroscopeDMA(IMU *imu) 
{
    uint8_t bmiStatus = 0;
    uint8_t dataSize = 6;
    uint8_t txData[dataSize];
    
    txData[0] = GYRO_X_LSB | 0x80;
    
    setCsPin(imu, GYROSCOPE, LOW);
    bmiStatus += HAL_SPI_TransmitReceive_DMA(imu->SPI, txData, imu->gyroDmaData, dataSize);
    setCsPin(imu, GYROSCOPE, HIGH);
    
    return bmiStatus;
}

uint8_t bmi088readGyroscopeDMAComplete(IMU *imu) 
{
    imu->x = (int16_t)((imu->gyroDmaData[0] << 8) | imu->gyroDmaData[1]) * imu->gyroScaleFactor;
    imu->y = (int16_t)((imu->gyroDmaData[2] << 8) | imu->gyroDmaData[3]) * imu->gyroScaleFactor;
    imu->z = (int16_t)((imu->gyroDmaData[4] << 8) | imu->gyroDmaData[5]) * imu->gyroScaleFactor;

    return HAL_OK;
}