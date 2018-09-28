#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

bool mpu_ready = false;
uint8_t mpuIntStatus;
uint16_t fifoCount;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;

MPU6050 mpu;

void mpu_init(){
  
  mpu.initialize();
  
  bool devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(GYRO_X_OFFSET);
  mpu.setYGyroOffset(GYRO_Y_OFFSET);
  mpu.setZGyroOffset(GYRO_Z_OFFSET);
  mpu.setXAccelOffset(ACC_X_OFFSET);
  mpu.setYAccelOffset(ACC_Y_OFFSET);
  mpu.setZAccelOffset(ACC_Z_OFFSET);

  #ifdef DEBUG_
  mpu.setRate(11);
  #else
  mpu.setRate(0);
  #endif
  mpu.setDLPFMode(1);

  if (devStatus == 0){
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    #ifdef DEBUG_
    Serial.println(F("MPU begin OK!"));
    #endif
    mpu_ready = true;
  }else{
    mpu_ready = false;
    
    #ifdef DEBUG_
    Serial.print(devStatus);
    Serial.println(F("MPU begin BAD"));
    #endif
    
    while(1){
      scream_blocking(100, 2000);
    }
  }

  
//  longs[0] = mpu_ready; // DEBUGGING
}
void mpu_update(){
  if (!mpu_ready) return;

  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();

    scream_once(30);
    #ifdef DEBUG_
    Serial.println(F("FIFO overflow"));
    #endif
  }else if (mpuIntStatus & 0x02) {
    if (fifoCount >= packetSize){
      
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu_updated = true;
      
      // Calculate everything
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      //Rearranging, swapping pitch and roll
      auto pitch = ypr[2];
      ypr[2] = ypr[1];
      ypr[1] = pitch;
      //Fixing values, for all right pitch and yaw
      ypr[1] = -ypr[1];
      ypr[2] = -ypr[2];
      
      for(int i=0;i<3;i++){
        ypr[i]*=RadToDeg;
//        longs[i] = (long)ypr[i];
      } // Convert to degrees
      
    }
  }
}
