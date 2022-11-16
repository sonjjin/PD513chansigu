#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "kalman.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

int16_t AcX, AcY, AcZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle, accZangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
//Set up the ros node and publisher
//std_msgs::String imu_msg;
std_msgs::Float32 imu_accX;
std_msgs::Float32 imu_accY;
std_msgs::Float32 imu_aglZ;
//std_msgs::Float32 imu_aglX;
//std_msgs::Float32 imu_aglY;
//ros::Publisher imu("imu", &imu_msg);
ros::Publisher accX("accX", &imu_accX);
ros::Publisher accY("accY", &imu_accY);
//ros::Publisher aglX("aglX", &imu_aglX);
//ros::Publisher aglY("aglY", &imu_aglY);
ros::Publisher aglZ("aglZ", &imu_aglZ);

ros::NodeHandle nh;
// MPU6050 mpu6050(Wire, 0.08, 0.92);
long publisher_timer;


void setup()
{
  nh.initNode();
//  nh.advertise(imu);
  nh.advertise(accX);
  nh.advertise(accY);
  //nh.advertise(aglX);
  //nh.advertise(aglY);
  nh.advertise(aglZ);
  Serial.begin(115200);
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x02; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  //0x00 : 256hz
  //0x01 : 199hz
  //0x02 : 98 hz
  //0x03 : 42 hz
  
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);

  }
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcY = ((i2cData[2] << 8) | i2cData[3]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(AcX, AcZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(AcY, AcZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(AcY, AcX) + PI) * RAD_TO_DEG;


  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  kalmanZ.setAngle(accZangle);

  gyroXangle = accXangle;
  gyroYangle = accYangle;
  gyroZangle = accZangle;


  timer = millis();
}

void loop()
{
  if (millis() > publisher_timer)
  {
      angleValue();    

      imu_accX.data = AcX;
      imu_accY.data = AcY;
      //imu_aglX.data = GyX;
      //imu_aglY.data = GyY;
      imu_aglZ.data = kalAngleZ;
      accX.publish(&imu_accX);
      accY.publish(&imu_accY);
      //aglX.publish(&imu_aglX);
      //aglY.publish(&imu_aglY);
      aglZ.publish(&imu_aglZ);
      publisher_timer = millis() + 100; //publish 100ms
      nh.spinOnce();
 
  }
}

void angleValue() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcY = ((i2cData[2] << 8) | i2cData[3]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(AcY, AcZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(AcX, AcZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(AcY, AcX) + PI) * RAD_TO_DEG;


  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  double gyroZrate = (double)gyroZ / 131.0;

  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
  gyroZangle += gyroZrate * ((double)(micros() - timer) / 1000000);



  // compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calculate the angle using a Complimentary filter
  // compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros() - timer) / 1000000);

  timer = micros();

  // temp = ((double)tempRaw + 12412.0) / 340.0;


}
