#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
int16_t AcX,AcY,GyZ;
//Set up the ros node and publisher
//std_msgs::String imu_msg;
std_msgs::Float32 imu_accX;
std_msgs::Float32 imu_accY;
std_msgs::Float32 imu_agl;
//ros::Publisher imu("imu", &imu_msg);
ros::Publisher accX("accX", &imu_accX);
ros::Publisher accY("accY", &imu_accY);
ros::Publisher agl("agl", &imu_agl);

ros::NodeHandle nh;
MPU6050 mpu6050(Wire, 0.08, 0.92);
long publisher_timer;


void setup()
{
  nh.initNode();
//  nh.advertise(imu);
  nh.advertise(accX);
  nh.advertise(accY);
  nh.advertise(agl);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.begin(9600);
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
      imu_agl.data = GyZ;
      accX.publish(&imu_accX);
      accY.publish(&imu_accY);
      agl.publish(&imu_agl);
      publisher_timer = millis() + 100; //publish ten times a second
      nh.spinOnce();
 
  }
}
