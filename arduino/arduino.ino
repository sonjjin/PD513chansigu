
/**
 * \file arduino_220902.ino
 *  
 * \brief 
 * Servo, Motor 테스트용 아두이노 파일
 * 
 * 코드분석 메모
 * <220902>
 * 12번 단자 Motor driver 어디핀에 연결되는지 확인 및 HIGH/LOW시 전진 후진 확인 필요(아마 High가 전진 일거 같음)
 * Servo duty 별 각도 제한 확인 1000~1500~2000us / 주기 : 20ms(50Hz) 제어 가능여부 확인
 * 
 */
 

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>


/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/* Pin Map Setting */
#define PIN_SERVO       9    // PWM Analog 출력 - Timer1, 490Hz
#define PIN_MOTOR_PWM   11   // PWM Analog 출력 - Timer2, 490Hz
#define PIN_MOTOR_DIR   12   // Digital 신호 출력
#define PIN_ENCODER0_A  2    // interupt 0
#define PIN_ENCODER0_B  3    // interupt 1

/* Arduino Configuration */
#define LOOP_TIME       10   // 10ms

/* Sensor & Actuator Configuration */
#define ENC_SPD_GAIN      0.0001
#define MOTOR_IN_MAX      255        // Pwm Duty 만들어주는 analogWrite 범위가 0~255일 건데 왜 300?
#define SERVO_MIN         40         // TODO : 스펙보고 확인 필요 / Servo 최소 각도 [deg]
#define SERVO_MAX         140        // Servo 최대 각도 [deg]
#define STEER_ANGLE_MAX   20         // 대략적인 Steering Angle[deg] Max

/* Controller Gains */
#define PID_GAIN_P    1
#define PID_GAIN_D    0.0015
#define PID_GAIN_I    0.01

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

int setSteer(float steerIn);

void setMotorPwm(int input);
void runPIDCtrl(float ref);

void doEncoderA();
void doEncoderB();

float getVehicleSpeed();
void  ctrl_servo_cb(const std_msgs::Float32 & ctrl_servo_msg);
void  ctrl_motor_cb(const std_msgs::Float32 & ctrl_motor_msg);

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

/*  
* TODO : Servo 각도 배열
*   각도 범위   : ?? (40~140[deg])
*   각도당 Duty : ?? (1000~2000us (1500us = 90deg / 직진))
*/ 
int servoList[11] = {SERVO_MIN, 50, 60, 70, 80, 90, 100, 110, 120, 130, SERVO_MAX};      // 수정된 servo list
float steerList[11] = {-0.40697,-0.37506,-0.31587,-0.23214,-0.16328,-0.017378,0.128524,0.178899,0.30116,0.384688,0.41333};  // 수정된 steer list
Servo servo;          // servo용 객체

// 시간 측정용
unsigned long prev_time_ms;
unsigned long count=0;    // 시작 후 10ms당 1번씩 count

// 차량 제어용 변수들
int servoInput = 0;     
float rosInput_servo=0;   // [deg]

boolean motorRunStatus;
float refMotorSpeed;
int   motorPwmDuty;
int   encoder0Pos = 0;
float rosInput_refMotorSpeed;   // [refMotor Pwm Duty -255~255]

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32> sub_ctrl_servo("ctrl_servo", ctrl_servo_cb);
ros::Subscriber<std_msgs::Float32> sub_ctrl_motor("ctrl_motor", ctrl_motor_cb);

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

void setup(){
  /*
  * 모터 드라이버 input용 PWM 주파수 변경
  *   Pin 11(Timer2, 490Hz)의 주파수 변경 :-> 31.37255 [kHz] 
  *   모터 드라이버(DMC-16) 권장 Input PWM 주파수 : 1~40kHz
  *   
  *   주파수 계산 방식 : clk_T2S=31.37255 [kHz] / Prescaler 
  *   Prescaler 설정 : TCCR2B(2:0 bit) : 64 -> 1로 바꿈)
  *   
  *   reference : https://gist.github.com/anbara/e7e371d4fbdff896e8703fdb000fdaeb
  *             : ATmega328P_Datasheet, 17.10 / 17.11.2 참조
  */ 
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;      
  
  /* Servo Setup */
  servo.attach(PIN_SERVO);         // Servo Pin 지정
  servo.writeMicroseconds(1500);  // Servo 위치 90도로 초기화
  
  /* Motor Driver Setup */
  pinMode(PIN_MOTOR_PWM, OUTPUT);   // Motor Pwm pin 지정, analogWrite 쓸 때 자동 지정되긴 함
  pinMode(PIN_MOTOR_DIR, OUTPUT);   // Motor 회전방향
  
  /* Encoder Driver Setup */
  pinMode(PIN_ENCODER0_A, INPUT);   // PIN_ENCODER0_A 읽는 것
  pinMode(PIN_ENCODER0_B, INPUT);   // PIN_ENCODER0_B 읽는 것
  attachInterrupt(0, doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)

//  Serial.begin (115200);    // 아두이노 시리얼 통신
  prev_time_ms = millis();    // setup시 시간 측정  
  setSteer(0.0);     
  setMotorPwm(0);
  nh.initNode();
  nh.subscribe(sub_ctrl_servo);
  nh.subscribe(sub_ctrl_motor);
}

/* Arduino Loop */
void loop() {  
  // subscriber들이 한번 정보를 받도록 함
  nh.spinOnce();

  if((millis() - prev_time_ms)>=(unsigned long)(10)){    
    // 10ms에 한번 제어하도록 함
    setSteer(rosInput_servo*DEG_TO_RAD);

    float dt = millis() - prev_time_ms;
    float vehicleSpeed = getVehicleSpeed(dt);
    
    runPIDCtrl(rosInput_refMotorSpeed, vehicleSpeed);
    count++;
    prev_time_ms = millis();  // loop 끝의 시간 확인
  }
}

/**
 * steering angle[rad]을 servoOut 으로 변환 (원본에서 수정됨)
 *
 * @param   steerIn   조향 각도[rad]
 * @return  servoOut  40도() ~ 140도
 */
int setSteer(float steerIn){
  int numList = 11;
  int servoOut= 90;

  if(steerIn<steerList[0]){
    servoOut = servoList[0];
  }
  else if (steerIn>steerList[numList-1]){
    servoOut = servoList[numList-1];
  }
  else{
    for(int index=0; index<numList-1; index++){
      if((steerList[index]<=steerIn)&&(steerIn<steerList[index+1])){
        servoOut = (int)((10.00)*(steerIn-steerList[index])/(steerList[index+1]-steerList[index]))+servoList[index];
        break;
      }
    }
  }
  servo.write(servoOut);
}

/**
 * Motor Pwm Duty 0~255 Setting 하는것
 *
 * @param   input     PWM Duty [-255~255]
 *                              -255~0은 후진으로 Setting
 *                              0~255는 전진으로 Setting
 */
void setMotorPwm(int input)
{ 
  motorPwmDuty = input;
  // Serial.println("setPWM");
  if((input >= (-1*MOTOR_IN_MAX)) && (input <= MOTOR_IN_MAX))
  // if input signal is in range
  {
    if(input>=0)
    {
      digitalWrite(PIN_MOTOR_DIR,HIGH);
      analogWrite(PIN_MOTOR_PWM,input);
    }
    else    // input < 0
    {
      digitalWrite(PIN_MOTOR_DIR,LOW);
      analogWrite(PIN_MOTOR_PWM,(-input));
    }
  }
}

void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(PIN_ENCODER0_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(PIN_ENCODER0_B) == LOW) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(PIN_ENCODER0_B) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  //Serial.println (encoder0Pos);
}

/* EncoderB Interrupt - 출력파형 확인 */
void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(PIN_ENCODER0_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(PIN_ENCODER0_A) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(PIN_ENCODER0_A) == LOW) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  //Serial.println (encoder0Pos);
}


/**
 * Motor Pwm Duty 0~255 Setting 하는것
 *
 * @param   ref           reference vehicle speed PWM Duty [-255~255]
 * @param   vehicleSpeed  Encoder를 통해 계산된 vehicle Speed
 *                              0~255는 전진으로 Setting
 */
void runPIDCtrl(float ref, float vehicleSpeed){
  // Error 값 초기화
  float error = ref - vehicleSpeed;
  static float iValue = 0;        // static 선언으로 처음만 초기화되고 안없어짐
  static float prevErr = 0;

  // 미분, 적분오차 계산
  float dValue = (prevErr - error) / (0.001*LOOP_TIME);
  iValue = (iValue*0.97) + error * (0.001*LOOP_TIME); // integral with forgetting factor

  // PID 제어값 계산 및 PWM 제어기에 넣어주기
  float u = PID_GAIN_P * error + PID_GAIN_I * iValue + PID_GAIN_D * dValue;
  
  Serial.print("Motor PWM Set:");
  Serial.println(u);

  setMotorPwm((int)u);

  // 이전 Error 값 저장
  prevErr = error;
}

float getVehicleSpeed(float dt){
  float speed = ENC_SPD_GAIN * encoder0Pos / (0.001*dt);    // encoder 이동 + Speedgain으로 변위 측정 / LOOP_TIME (10ms 이므로 0.01 sec로 변환)
  //Serial.println(speed);
  encoder0Pos = 0;
  return speed;
}

void  ctrl_servo_cb(const std_msgs::Float32 &ctrl_servo_msg){
  // Float32 스티어링 각도 [deg]
  rosInput_servo = ctrl_servo_msg.data;
}

void  ctrl_motor_cb(const std_msgs::Float32 &ctrl_motor_msg){
  // Float32 motor duty [0~255]
  rosInput_refMotorSpeed = ctrl_motor_msg.data;
}
//void GetMessageServo( const std_msgs::Float32 & servo_msg){
//  rosinput_servo = servo_msg.data;
//  servoInput = setSteer(rosinput_servo);
//}
