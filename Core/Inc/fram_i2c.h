/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_FRAM_I2C_H_
#define INC_FRAM_I2C_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h" // Để có kiểu dữ liệu HAL_StatusTypeDef và I2C_HandleTypeDef

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FRAM_I2C_DEVICE_ADDRESS    (0x50 << 1) // Địa chỉ thiết bị I2C 7-bit của FRAM, dịch trái 1
#define FRAM_MAX_MEMORY_ADDRESS    0x07FF      // Địa chỉ ô nhớ tối đa cho 16Kbit FRAM (2047)
#define FRAM_DEFAULT_TIMEOUT     100         // Timeout mặc định cho các thao tác I2C (ms)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Khởi tạo module FRAM và kiểm tra thiết bị sẵn sàng.
  * @param hi2c: Con trỏ tới handle I2C_HandleTypeDef đã được khởi tạo.
  * @retval HAL_StatusTypeDef: HAL_OK nếu khởi tạo và thiết bị sẵn sàng, ngược lại là lỗi.
  */
HAL_StatusTypeDef FRAM_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief Ghi một mảng byte vào FRAM từ một địa chỉ.
  * @param hi2c: Con trỏ tới handle I2C.
  * @param memAddress: Địa chỉ ô nhớ bắt đầu ghi.
  * @param pData: Con trỏ tới dữ liệu cần ghi.
  * @param size: Số lượng byte cần ghi.
  * @retval HAL_StatusTypeDef: Trạng thái HAL.
  */
HAL_StatusTypeDef FRAM_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size);

/**
  * @brief Đọc một mảng byte từ FRAM bắt đầu từ một địa chỉ.
  * @param hi2c: Con trỏ tới handle I2C.
  * @param memAddress: Địa chỉ ô nhớ bắt đầu đọc.
  * @param pData: Con trỏ tới buffer để lưu dữ liệu đọc được.
  * @param size: Số lượng byte cần đọc.
  * @retval HAL_StatusTypeDef: Trạng thái HAL.
  */
HAL_StatusTypeDef FRAM_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *pData, uint16_t size);

#endif /* INC_FRAM_I2C_H_ */