#include "LSM9DS1.h"


void resetAndCheck(void){
	uint8_t id[2];
	uint8_t who_am_i = 15 | 0x80;
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, &who_am_i, (uint8_t *) &id, 2, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	//int id = read8(LSM9DS1_REGISTER_WHO_AM_I_XG);
	printf("ID[0]: %d\n\r", id[0]);
	printf("ID[1]: %d\n\r", id[1]);
}

void readGyro(void) {
  uint8_t buffer[6];

  readBuffer(LSM9DS1_REGISTER_OUT_X_L_XL, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  accelData.x = xhi;
  accelData.y = yhi;
  accelData.z = zhi;
}

void setupGyro(lsm9ds1AccelRange_t range) {
	  uint8_t reg = read8(0x80 | LSM9DS1_REGISTER_CTRL_REG6_XL);

	  reg &= ~(0b00011000);
	  reg |= range;
	  //Serial.println("set range: ");
	  write8(LSM9DS1_REGISTER_CTRL_REG6_XL, reg);

	  switch (range)
	  {
	    case LSM9DS1_ACCELRANGE_2G:
	      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
	      break;
	    case LSM9DS1_ACCELRANGE_4G:
	      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
	      break;
	    case LSM9DS1_ACCELRANGE_8G:
	      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
	      break;
	    case LSM9DS1_ACCELRANGE_16G:
	      _accel_mg_lsb =LSM9DS1_ACCEL_MG_LSB_16G;
	      break;
	  }
}

uint8_t read8(uint8_t reg) {
	uint8_t value;

	uint8_t read_reg[1];
	read_reg[0] = reg | 0x80;

	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, read_reg, (uint8_t *) &value, 1, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	return value;
}
void write8(uint8_t reg, uint8_t value) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

	uint8_t write_reg[0];
	write_reg[0] = reg&0x7F;

	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, write_reg, 1,1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	vTaskDelayUntil(&xLastWakeTime, 100);

	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,  value, sizeof(value),1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

}
uint8_t readBuffer(uint8_t reg, uint8_t len, uint8_t *buffer) {

	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *) &reg, 1, 1000);
	HAL_SPI_Receive(&hspi2, buffer, len, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	return len;
}


