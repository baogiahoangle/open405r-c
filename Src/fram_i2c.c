/* Includes ------------------------------------------------------------------*/
#include "fram_i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

HAL_StatusTypeDef FRAM_Init(I2C_HandleTypeDef *hi2c)
{
  return HAL_I2C_IsDeviceReady(hi2c, FRAM_I2C_DEVICE_ADDRESS, 2, 10);
}

HAL_StatusTypeDef FRAM_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
  if ((memAddress + size - 1) > FRAM_MAX_MEMORY_ADDRESS) {
      return HAL_ERROR; 
  }
  return HAL_I2C_Mem_Write(hi2c, FRAM_I2C_DEVICE_ADDRESS, memAddress, I2C_MEMADD_SIZE_16BIT, pData, size, FRAM_DEFAULT_TIMEOUT);
}

HAL_StatusTypeDef FRAM_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
  if ((memAddress + size - 1) > FRAM_MAX_MEMORY_ADDRESS) {
      return HAL_ERROR;
  }
  return HAL_I2C_Mem_Read(hi2c, FRAM_I2C_DEVICE_ADDRESS, memAddress, I2C_MEMADD_SIZE_16BIT, pData, size, FRAM_DEFAULT_TIMEOUT);
}