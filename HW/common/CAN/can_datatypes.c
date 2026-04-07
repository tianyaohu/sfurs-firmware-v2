#include "can_datatypes.h"

HAL_StatusTypeDef can_motor_write_speed_ramp(FDCAN_HandleTypeDef* can_pointer,
		FDCAN_TxHeaderTypeDef* tx_header,
		uint8_t* tx_data,
		uint8_t* motor_id,
		int32_t speed,
		uint32_t time)
{
	  tx_header.Identifier = (SPEED_WRITE | (motor_id & 0x7F));
	  uint32_t payload = ((((uint32_t)(uint16_t)time) << 16) | (uint16_t)speed);
	  tx_data[0] = (test_data >> 24) & 0xFF;
	  tx_data[1] = (test_data >> 16) & 0xFF;
	  tx_data[2] = (test_data >> 8) & 0xFF;
	  tx_data[3] = test_data & 0xFF;
	  return HAL_FDCAN_AddMessageToTxFifoQ(can_pointer, tx_header, tx_data);
}
