#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
int16_t AcX,AcY,GyZ, GyX, GyY;
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
MPU6050 mpu6050(Wire, 0.08, 0.92);
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
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.begin(115200);
}

void loop()
{
  mpu6050.update();
  if (millis() > publisher_timer)
  {
      mpu6050.update();
      AcX = round(mpu6050.getAccX()*100);
      AcY = round(mpu6050.getAccY()*100);
      //AcZ = round(mpu6050.getAccZ()*100);
      //GyX = round(mpu6050.getAngleX()*100);
      //GyY = round(mpu6050.getAngleY()*100);
      GyZ = round(mpu6050.getAngleZ()*100);
      //Serial.println(AcX);
      //Serial.println(AcY);
      //Serial.println(AcZ);
      //Serial.println(GyX);
      //Serial.println(GyY);
      //Serial.println(GyZ);
      
      //String AX = String(AcX);
      //String AY = String(AcY);
      //String AZ = String(AcZ);
      //String GX = String(GyX);
      //String GY = String(GyY);
      //String GZ = String(GyZ);
      
      //String tmp = String(Tmp);
      //String data = "A" + AX + "B"+ AY + "C" + GZ + "D" ;
      //Serial.println(String(AcX));
      imu_accX.data = AcX;
      imu_accY.data = AcY;
      //imu_aglX.data = GyX;
      //imu_aglY.data = GyY;
      imu_aglZ.data = GyZ;
      accX.publish(&imu_accX);
      accY.publish(&imu_accY);
      //aglX.publish(&imu_aglX);
      //aglY.publish(&imu_aglY);
      aglZ.publish(&imu_aglZ);
      publisher_timer = millis() + 100; //publish 100ms
      nh.spinOnce();
 
  }
}
