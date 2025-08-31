/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_FRAM_I2C_H_
#define INC_FRAM_I2C_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h" // For HAL_StatusTypeDef and I2C_HandleTypeDef types

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FRAM_I2C_DEVICE_ADDRESS    (0x50 << 1) // 7-bit I2C device address of FRAM, shifted left by 1
#define FRAM_MAX_MEMORY_ADDRESS    0x07FF      // Maximum memory address for 16Kbit FRAM (2047)
#define FRAM_DEFAULT_TIMEOUT       100         // Default timeout for I2C operations (ms)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Initialize the FRAM module and check if the device is ready.
  * @param hi2c: Pointer to the initialized I2C_HandleTypeDef handle.
  * @retval HAL_StatusTypeDef: HAL_OK if initialized and device is ready, otherwise error.
  */
HAL_StatusTypeDef FRAM_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief Write an array of bytes to FRAM starting from a specific address.
  * @param hi2c: Pointer to the I2C handle.
  * @param memAddress: Start memory address to write.
  * @param pData: Pointer to the data to be written.
  * @param size: Number of bytes to write.
  * @retval HAL_StatusTypeDef: HAL status.
  */
HAL_StatusTypeDef FRAM_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size);

/**
  * @brief Read an array of bytes from FRAM starting from a specific address.
  * @param hi2c: Pointer to the I2C handle.
  * @param memAddress: Start memory address to read.
  * @param pData: Pointer to the buffer to store read data.
  * @param size: Number of bytes to read.
  * @retval HAL_StatusTypeDef: HAL status.
  */
HAL_StatusTypeDef FRAM_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size);

#endif /* INC_FRAM_I2C_H_ */