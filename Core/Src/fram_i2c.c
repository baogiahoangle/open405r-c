/* Includes ------------------------------------------------------------------*/
#include "fram_i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/**
  * @brief Khởi tạo module FRAM và kiểm tra thiết bị sẵn sàng.
  */
HAL_StatusTypeDef FRAM_Init(I2C_HandleTypeDef *hi2c)
{
  // Kiểm tra xem thiết bị FRAM có phản hồi trên bus I2C không.
  // Thử gửi địa chỉ thiết bị với 2 lần thử, timeout 10ms mỗi lần.
  return HAL_I2C_IsDeviceReady(hi2c, FRAM_I2C_DEVICE_ADDRESS, 2, 10);
}

/**
  * @brief Ghi một mảng byte vào FRAM từ một địa chỉ.
  */
HAL_StatusTypeDef FRAM_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
  if ((memAddress + size - 1) > FRAM_MAX_MEMORY_ADDRESS) {
      return HAL_ERROR; // Ghi vượt quá giới hạn bộ nhớ
  }
  return HAL_I2C_Mem_Write(hi2c, FRAM_I2C_DEVICE_ADDRESS, memAddress, I2C_MEMADD_SIZE_16BIT, pData, size, FRAM_DEFAULT_TIMEOUT);
}

/**
  * @brief Đọc một mảng byte từ FRAM bắt đầu từ một địa chỉ.
  */
HAL_StatusTypeDef FRAM_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size)
{
  if ((memAddress + size - 1) > FRAM_MAX_MEMORY_ADDRESS) {
      return HAL_ERROR; // Đọc vượt quá giới hạn bộ nhớ
  }
  return HAL_I2C_Mem_Read(hi2c, FRAM_I2C_DEVICE_ADDRESS, memAddress, I2C_MEMADD_SIZE_16BIT, pData, size, FRAM_DEFAULT_TIMEOUT);
}