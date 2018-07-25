#include "Arduino.h"
#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18


typedef struct imu_si_data{
  float ax;
  float ay;
  float gz;
};

typedef imu_si_data imu_data;


class mpu_9250_lite{
	private:
		imu_data imu_sense;
		imu_data imu_offset;

		void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
		void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
		void remove_gyro_offset(void);
    imu_data read_imu(void);

	public:
		imu_si_data get_imu_SI(void);
		mpu_9250_lite(void);

};
 