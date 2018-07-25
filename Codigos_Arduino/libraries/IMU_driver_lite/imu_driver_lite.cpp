#include "imu_driver_lite.h"


//Funcion auxiliar lectura
void mpu_9250_lite::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();
}
 
 
// Funcion auxiliar de escritura
void mpu_9250_lite::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}

imu_data mpu_9250_lite::read_imu(void){  
   // ---  Lectura acelerometro y giroscopio --- 
   uint8_t Buf[14];
   imu_data temp;
   
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);   
    
   // Convertir registros acelerometro
   temp.ax = (-(Buf[0] << 8 | Buf[1]))-imu_offset.ax;
   temp.ay = (-(Buf[2] << 8 | Buf[3]))-imu_offset.ay;
  
   temp.gz = -((Buf[12] << 8 | Buf[13])-imu_offset.gz);

 /*
   // ---  Lectura del magnetometro --- 
   uint8_t ST1;
 /*  do
   {
    Serial.println("llego");
      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
   } while (!(ST1 & 0x01));
 
   uint8_t Mag[7];
   I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
 
 
   // Convertir registros magnetometro
   int16_t mx = -(Mag[3] << 8 | Mag[2]);
   int16_t my = -(Mag[1] << 8 | Mag[0]);
   int16_t mz = -(Mag[5] << 8 | Mag[4]);*/
    return (temp);
}

void mpu_9250_lite::remove_gyro_offset(void){  

  imu_data temp;
  imu_data temp2;
  
  temp2.ax=0;
  temp2.ay=0;
  temp2.gz=0;
  int old_time =millis();
  Serial.print("Calibrando offset giroscopio y acelerometro, no mover");
  for (int i=0;i<100;i++){
    while(millis()-old_time<100){
      
    }
    old_time =millis();
    imu_data temp=read_imu();
    temp2.gz+=temp.gz;
    temp2.ax+=temp.ax;
    temp2.ay+=temp.ay;
    Serial.print(i);
    Serial.println("%");
        
  }
  Serial.println("Listo.");
  imu_offset.gz=temp2.gz/100;  
  imu_offset.ax=temp2.ax/100;
  imu_offset.ay=temp2.ay/100;


}

imu_si_data mpu_9250_lite::get_imu_SI(void){ // Devuelve en grados y en m/s2
  imu_data temp = read_imu() ;
  imu_si_data temp_SI;
  
  temp_SI.gz=temp.gz*2000.0f/32767.5f;

  temp_SI.ax=temp.ax*9.8*16.0f/32767.5f;
  temp_SI.ay=temp.ay*9.8*16.0f/32767.5f;
  return(temp_SI);
}

mpu_9250_lite::mpu_9250_lite(void){
  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
  // Configurar giroscopio
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configurar magnetometro
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
  imu_offset.gz=0;
  remove_gyro_offset();  

  Serial.println("Ready IMU");
}