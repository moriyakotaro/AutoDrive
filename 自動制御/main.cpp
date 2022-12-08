#include <Arduino.h>
#include "drive.h"
#include "CAN.h"
#include "RobomasMotor.h"
#include "Encoder.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Metro.h>
#include <SPI.h>
#include "DualShock.h"
#include "Drive_.h"
#include "function.h"
#include "FlexCAN.h"
//#include "CAN.h"
//#include "RobomasMotor.h"
#include "define.h"
#define HWSERIAL Serial1
// #define PI 3.14159265358979

#define MOTOR_COUNT 4
#define MOTOR_CANBUS 0
#define COMMAND_CANBUS 1

#define CONTROL_CYCLE 1.0 /*ms*/
#define MOTOR_CYCLE 2.0

#define SMOOTH_SLOW 1
#define SMOOTH_NORMAL 10
#define RISING_PWM 0
#define SMALL_PWM 10

#define ENCODER_X_A 9
#define ENCODER_X_B 7
#define ENCODER_Y_A 26
#define ENCODER_Y_B 27

#define LED1 2
#define GYAIRO_LED 24
#define BUTTON 13
#define BUTTON_PUSH 0

#define IS(x) digitalRead(x) == BUTTON_PUSH

#define GYAIRO_CYCLE 1
#define ENCODER_CYCLE 1
#define DISP_CYCLE 10
#define LED_CYCLE 500

short drive_gain[MOTOR_COUNT][3] = {
  { 1,-1,-1},
  { 1, 1,-1},
  {-1, 1,-1},
  {-1,-1,-1},
};

double drive_PIDx_gain[3] = {9.0,0.0,0};
double drive_PIDy_gain[3] = {9.0,0.0,0};
double drive_PIDr_gain[3] = {1.7,0.01,0.5};

double M_Pg = 5.342;
double M_Ig = 4.21;
double M_Dg = 2.4;

double targets_rpm[4] = {0};//速さ(Hz)
double gyro_sence;
double x_val;
double y_val;
double x_keep[2];
double y_keep[2];
double lx = 0;
double ly = 0;
double rx = 0;
double lxbf = 0;
double lybf = 0;
double rxbf = 0;
double lxaf = 0;
double lyaf = 0;
double rxaf = 0;
// static int flag = 0;



////////////////////////////////////////
bool rotation_sign;
int8_t rotation_num=0;
double rotation_delta=0;
float rotation_keep[10];
//変更点
// uint8_t tx_data[8] = {0};
CAN_message_t sendM;

int data[10] = {0};
int monitoring = 0; 

// const int debugLED = 25;
uint8_t led_count = 0;
// int led_count = 0;
// int spd = 0;
/////////////////////////////////////




CanControl* drive_can = CanControl::CreateCanControl(MOTOR_CANBUS);
//CanControl* command_can = CanControl::CreateCanControl(COMMAND_CANBUS);
Motor motor(MOTOR_COUNT,SMOOTH_SLOW,SMOOTH_NORMAL,RISING_PWM,SMALL_PWM);
Drive AutoDrive(motor,CONTROL_CYCLE/1000);
DJIMotor SpdControl(drive_can,CONTROL_CYCLE/1000,MOTOR_COUNT);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

Encoder enc_x(ENCODER_X_A,ENCODER_X_B);
Encoder enc_y(ENCODER_Y_A,ENCODER_Y_B);

Metro controlTimming(CONTROL_CYCLE);
Metro motorTimming(MOTOR_CYCLE);
Metro gyaroTimming(GYAIRO_CYCLE);
Metro encoderTimming(ENCODER_CYCLE);
Metro dispTimming(DISP_CYCLE);
Metro ledTimming(LED_CYCLE);
Metro Serialtiming = Metro(1);
Metro idoutiming(4000);///////////
//変更点


void pinModeSet();
void getpalam();
void getEncorder();
void printD();

void timer(void);
void control_data_receive(int recive);

void setup() {
  delay(500);
  Serial.begin(38400);
  // Serial.begin(9600);
  HWSERIAL.begin(9600);
  SpdControl.init();
  pinModeSet();
  if(!bno.begin())while(1);
  delay(500);
  bno.setExtCrystalUse(true);
  for(int i=0;i<MOTOR_COUNT;i++)
  AutoDrive.setDriveGain(drive_gain);
  AutoDrive.setDrivePID(drive_PIDx_gain,drive_PIDy_gain,drive_PIDr_gain);
  for(int i=1;i<=MOTOR_COUNT;i++){
    SpdControl.setPIDgain(i,M_Pg,M_Ig,M_Dg);
  }
  delay(500);
  // delay(100);
  //変更点
}

void pinModeSet(){
  pinMode(LED1,OUTPUT);
  pinMode(GYAIRO_LED,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);
}
int con=0;
double idou[5][3] = {{0,0,0},{45,0,PI/2},{45,45,PI},{0,45,PI/2},{0,0,0}};
int c=0;
int m=-1; 
// Motor mm = Motor(DRIVEN_MOTOR_NUM,1,10,24,10);
// Drive d = Drive(mm, 10/1000);
// Drive d = new Motor(mm);
// classの中にclass
double gyro_z = 0;//旋回角の変数
int enc[2] = {};//エンコーダの配列

void loop() {
  int incomingByte;

  if (HWSERIAL.available() > 0) {
            incomingByte = HWSERIAL.read();
            
            control_data_receive(incomingByte);
            // Serial.write(10);
            // Serial.print("USB received: ");
            // Serial.print(incomingByte, DEC);
            // Serial.print(incomingByte);
            // HWSERIAL.write(ss++);
            // if(ss>10000)ss=0;
            
    }


  AutoDrive.now.r = gyro_sence;
	AutoDrive.searchPosition(x_val,y_val);

  // AutoDrive.to = Point(45,0,0); //目標値代入
  //AutoDrive.to = Point(45,45,0);



// if(abs(lxaf)>abs(lxbf))lx += lxaf-lxbf;
// if(abs(lyaf)>abs(lybf))ly += lyaf-lybf;
// if(abs(rxaf)>abs(rxbf))rx += rxaf-rxbf;
  
  

//   if(flag > 0)flag++;
//   // if(flag == 10)flag = 0;
//   if(flag == 0){
//     lxbf = joystick_lx;
//     lybf = joystick_ly;
//     rxbf = joystick_rx/100*2*PI;
//     flag++;
//   }

//   if(flag == 2){
//     lxaf = joystick_lx;
//     lyaf = joystick_ly;
//     rxaf = joystick_rx/100*2*PI;
//     flag = 0;
//   }
//   // lxaf = joystick_lx;
//   // lyaf = joystick_ly;
//   // rxaf = joystick_rx/100*2*PI;

//   AutoDrive.to = Point(lx,ly,rx);


  // (0,45,-PI/2);

  AutoDrive.to = Point(idou[c][0],idou[c][1],idou[c][2]);
    //if(c>)motor.stop_flag=true;

  if(idoutiming.check()){
    if(c == 4)c=0;
    if(c<4)c++;
  }
    AutoDrive.absoluteMove();
    AutoDrive.update();

    //旋回角を取得する
    //エンコーダの値を取得する
    // d.now.r = gyro_z; //ここで旋回角を代入する
    // d.searchPosition(enc[0],enc[1]);
    // d.to = Point(1000,1000,PI/2);
    // d.absoluteMove();
    // d.update();
    //ここにd.motor[i]の値でモーターを回す関数を書く






    delay(1);//変更点








////////////////////////////////////////////
  // if(idoutiming.check()){if(con)m++;
  // if(m>0&&c<4)c++;
  // }
/////////////////////////////////////////////

// anomalyDriveing(joystick_lx, -joystick_ly, joystick_rx, targets_rpm, gyro_sence);

  // AutoDrive.motor[1] = 1000;

  // if (HWSERIAL.available() > 0) {
  //           incomingByte = HWSERIAL.read();
            
  //           control_data_receive(incomingByte);
  //           // Serial.write(10);
  //           // Serial.print("USB received: ");
  //           // Serial.print(incomingByte, DEC);
  //           // Serial.print(incomingByte);
  //           // HWSERIAL.write(ss++);
  //           // if(ss>10000)ss=0;
  //           // anomalyDriveing(joystick_lx, -joystick_ly, joystick_rx, targets_rpm, gyro_sence);
            
  // }



//変更点////////////////////////////////////
  for(int i=1;i<=MOTOR_COUNT;i++){
    SpdControl.setTargetRPM(i,AutoDrive.motor[i-1]);
  }
// for(int i=1;i<=WHEEL_COUNT;i++){
//   // SpdControl.setTargetRPM(i,targets_rpm[i-1]);
//   SpdControl.setTargetRPM(i,0);
// }







///////////////////////////////////////////変更点
  // if(IS(BUTTON))con=1;
  // if(con)AutoDrive.update();
  // if(motorTimming.check()){
  // }
  if(controlTimming.check()){
    // if(con)AutoDrive.absoluteMove();
    // for(int i=1;i<=WHEEL_COUNT;i++){
    //   SpdControl.setTargetRPM(i,targets_rpm[i-1]);
    //   // SpdControl.setTargetRPM(i,0);
    // }
    SpdControl.speedControl();
    // spd = 0;
    // tx_data[6] = spd>>8;
    // tx_data[7] = spd&0xff;
    // //DriveCan->CANDataPush(0x200,tx_data);
    // //DriveCan->CANAllDataWrite();
            
    // sendM.id = 0x200;
    // sendM.len = 8;
    // sendM.buf[0] = 0;
    // sendM.buf[1] = 0;
    // sendM.buf[2] = 0;
    // sendM.buf[3] = 0;
    // sendM.buf[4] = 0;
    // sendM.buf[5] = 0;
    // sendM.buf[6] = tx_data[6];
    // sendM.buf[7] = tx_data[7];
  }
//////////////////////////////////////////////////






  getpalam();
  getEncorder();
  if(IS(BUTTON)){
  }
  if(dispTimming.check()){
    printD();
  }
  if(ledTimming.check()){
    digitalWrite(GYAIRO_LED,led_count++%2);
    // digitalWrite(debugLED,led_count++%2);
    if(led_count>1000)led_count=0;
  }
}


void getpalam(void){
  //Calibration status values: 0=uncalibrated, 3=fully calibrated
  imu::Vector<3> euler;
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  imu::Quaternion quat = bno.getQuat();
  euler = quat.toEuler();
  //get Axis Data

  double r=euler[0];
  rotation_keep[0] = r;
  rotation_sign = rotation_keep[9] <= rotation_keep[0];
  if(rotation_keep[0]*rotation_keep[9]<0){
    if(PI - fabs(rotation_keep[9]) < 0.1)rotation_sign = rotation_keep[9] > 0;
    //else if(fabs((rotation_num)*PI-gyro_sence) < 0.1)rotation_num += rotation_keep[1] < 0 ? 1:-1;
  }
  if(r*gyro_sence < 0 && fabs(r)>0.1){
    if(r>0)rotation_delta = -2*PI;
    else   rotation_delta = 2*PI ;
  }else{
    rotation_delta = 0;
  }
  
  gyro_sence = r + rotation_delta + 2*PI*rotation_num ;
  for(int i=9;i>0;i--){
    rotation_keep[i] = rotation_keep[i-1];
  }
  //gyro_sence=euler[0];
  //yaw -= byaw;//ヨー軸補正値;
  //get Calibration Data
  // C_system = system;
  // C_gyro = gyro;
  // C_accel = accel;
  // C_mag = mag;
  //delay(cyc_time);
}

void getEncorder(){
    x_keep[0] = -enc_x.read()/1000.0;
    x_val = x_keep[0] - x_keep[1];
    x_keep[1] = x_keep[0];
    y_keep[0] = enc_y.read()/1000.0;
    y_val = y_keep[0] - y_keep[1];
    y_keep[1] = y_keep[0];

}

void printD(){/*
    Serial.print(roll);
    
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");*/
    // Serial.print(lxbf);
    // Serial.print(",");
    // Serial.print(lybf);
    // Serial.print(",");
    // Serial.print(rxbf);
    // Serial.print(",,,");
    Serial.print(gyro_sence);
    Serial.print(",");
    Serial.print(targets_rpm[0]);
    Serial.print(",");
    Serial.print(targets_rpm[1]);
    Serial.print(",");
    Serial.print(targets_rpm[2]);
    Serial.print(",");
    Serial.print(targets_rpm[3]);
    // Serial.print(lxaf);
    // Serial.print(",");
    // Serial.print(lyaf);
    // Serial.print(",");
    // Serial.print(rxaf);
    // Serial.print(",,,");

    // Serial.print(lx);
    // Serial.print(",");
    // Serial.print(ly);
    // Serial.print(",");
    // Serial.print(rx);
    // Serial.print(",");
    // Serial.print(con);
    // Serial.print(" ");
    // Serial.print(gyro_sence);
    // Serial.print(" ");
    // Serial.print(AutoDrive.now.x);
    // Serial.print(" ");
    // Serial.print(AutoDrive.now.y);
    // Serial.print(" ");
    // Serial.print(AutoDrive.to.x);
    // Serial.print(" ");
    // Serial.print(AutoDrive.to.y);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.x,6);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.y,6);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.r,6);
    // Serial.print(" ");
    // for(int i=1;i<=4;i++){
    //   Serial.print(AutoDrive.motor[i-1]);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.angle[i]);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.ampare[i],10);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.ampare[i],10);
    //   Serial.print(" ");
    // }
      // Serial.print(AutoDrive.spdd[0],6);
      // Serial.print(" ");
    Serial.println();
}

void timer(void){
    monitoring++;
    if(monitoring>=200){
        circle_button   = 0;
        cross_button    = 0;
        triangle_button = 0;
        square_button   = 0;
        left_button  = 0;
        right_button = 0;
        up_button    = 0;
        down_button  = 0;
        l1_button = 0;
        l2_button = 0;
        l3_button = 0;
        r1_button = 0;
        r2_button = 0;
        r3_button = 0;
        ps_button = 0;
        start_button  = 0;
        select_button = 0;
        joystick_rx = 0;
        joystick_ry = 0;
        joystick_lx = 0;
        joystick_ly = 0;
    }
}
int No = 0;
void control_data_receive(int recive){
  //static int No;
//Serial.print(No);Serial.print(",");
// Serial.print(recive);
  if(recive == 0x80){
    No = 0;
    data[No++] = 0x80;
  }else if(No > 0){
    data[No++] = recive;
    if(No > 8){
      updataState(data);
    }
  }
}