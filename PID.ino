/******* using kalman filter to get the data from MPU 6050. Kalman filter is a kind of method to rectify the data from the MPU. The process of getting data has been proved right and reliable. Then I used the output of MPU to be the input of PID algorithm. I used PID library I found in Arduino. For now the problem is the PID can only work when the offset occurs when it is larger than the expected. It means when the quadcopter tilts to one side, the PID will not work. It only works when it tils to the right.******************************************/

/******************I do not take full credits of this code since the process of getting data is not written by me****************************************************************/

#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Math.h>
#include <Kalman.h>


#include <Servo.h> 
SoftwareSerial BTserial(10,11); // RX | TX
// Connect the HC-05 TX to Arduino pin 2 RX. 
// Connect the HC-05 RX to Arduino pin 3 TX through a voltage divider.

float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量

const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据

unsigned long nLastTime = 0; //上一次读数的时间
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
float fLastYaw=0.0f;//lasttime yaw
Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器
Kalman kalmanYaw;//Yaw角

Servo esc1, esc2,esc3,esc4;
  int mapmotor_0=0;
  int mapmotor_1=0;
  int mapmotor_2=0;
  int mapmotor_3=0;



//initialization for PID
unsigned int thr=500;
double motor_0, motor_1;
double motor_2, motor_3;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;

double Setpoint5, Input5, Output5;
float fNewPitch=0;
float fNewRoll=0;
float fNewYaw=0;
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=8, consKi=0.05, consKd=0.25;
float constant_roll=0,constant_pitch=0,constant_yaw=0;
PID PIDpitch1(&Input1,&Output1,&Setpoint1,consKp,consKi,consKd,DIRECT);
PID PIDroll1(&Input2,&Output2,&Setpoint2,consKp,consKi,consKd,DIRECT);

PID PIDyaw(&Input5,&Output5,&Setpoint5,consKp,consKi,consKd,DIRECT);



void setup() {
  // 
  Serial.begin(9600);
      BTserial.begin(9600);  //bluetooth
  Input1=fNewPitch;
  Input2=fNewRoll;
  Input5=fNewYaw;
  Setpoint1=0;
  Setpoint2=0;
  Setpoint5=-1;
  motor_0=0;
  motor_1=0;
  motor_2=0;
  motor_3=0;
  PIDpitch1.SetMode(AUTOMATIC);

  PIDroll1.SetMode(AUTOMATIC);

  PIDyaw.SetMode(AUTOMATIC);
  
    
    Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备

  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  float fYaw=realVals[2];

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  fNewYaw = kalmanYaw.getAngle(fYaw,realVals[6],dt);
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
  float fYawRate = (fNewYaw-fLastYaw)/dt;
 
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  fLastYaw=fNewYaw;
  constant_roll=fNewRoll;
  constant_pitch=fNewPitch;
  constant_yaw=fNewYaw;
  //calibrate for escs;
  esc1.attach(3);
  esc1.write(0);
  esc2.attach(5);
  esc2.write(0);
  esc3.attach(7);
  esc3.write(0);
  esc4.attach(9);
  esc4.write(0);
  calibrate();
  

}

void loop() {
  // 
  // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BTserial.available())
    {  
        thr= BTserial.parseInt();
        Serial.write(thr);
    }
    else
    {
      thr=thr;          
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-05

  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  float fYaw=realVals[2];

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt)-constant_roll;
  fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt)-constant_pitch;
  fNewYaw = kalmanYaw.getAngle(fYaw,realVals[6],dt)-constant_yaw;
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
  float fYawRate = (fNewYaw-fLastYaw)/dt;
 
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  fLastYaw=fNewYaw;
  //更新本次测的时间
  nLastTime = nCurTime;
  //向串口打印输出Roll角和Pitch角，运行时在Arduino的串口监视器中查看
  Serial.print("Roll:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")\n");
  Serial.print(fNewYaw); Serial.print('(');
  Serial.print(fYawRate); Serial.print(")\n");
  //PID part
  
  Input1=fNewPitch;

  Input2=fNewRoll;

  Input5=fNewYaw;
  if (fNewPitch<0)
  {
    PIDpitch1.SetControllerDirection(DIRECT);
    motor_0=0-Output1;
    motor_1=0+Output1;
    motor_2=0-Output1;
    motor_3=0+Output1;
  }
  if (fNewPitch>0)
  {
    PIDpitch1.SetControllerDirection(REVERSE);
    motor_0=0+Output1;
    motor_1=0-Output1;
    motor_2=0+Output1;
    motor_3=0-Output1;
  }
  if (fNewRoll<0)
  {
    PIDroll1.SetControllerDirection(DIRECT);
    motor_0=motor_0-Output2;
    motor_1=motor_1-Output2;
    motor_2=motor_2+Output2;
    motor_3=motor_3+Output2;
  }
  if (fNewRoll>0)
  {
    PIDroll1.SetControllerDirection(REVERSE);
    motor_0=motor_0+Output2;
    motor_1=motor_1+Output2;
    motor_2=motor_2-Output2;
    motor_3=motor_3-Output2;
  }
  PIDpitch1.Compute();

  PIDroll1.Compute();

  PIDyaw.Compute();
  mapmotor_0=map(motor_0,-510,510,0,1000)+thr;
  mapmotor_1=map(motor_1,-510,510,0,1000)+thr;
  mapmotor_2=map(motor_2,-510,510,0,1000)+thr;
  mapmotor_3=map(motor_3,-510,510,0,1000)+thr;
  Serial.print("mapmotor_0 mapmotor_1\t");
  Serial.print(mapmotor_0);
  Serial.print("\t");
  Serial.print(mapmotor_1);
  Serial.print("\n");
  Serial.print("mapmotor_2 mapmotor_3\t");
  Serial.print(mapmotor_2);
  Serial.print("\t");
  Serial.print(mapmotor_3);
  Serial.print("\n");
  esc1.write(mapmotor_0);
  esc2.write(mapmotor_1);
  esc3.write(mapmotor_2);
  esc4.write(mapmotor_3);
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}
void calibrate() {
  esc1.write(2000);
  esc2.write(2000);
  esc3.write(2000);
  esc4.write(2000);
  delay(3000);
  esc1.write(700);
  esc2.write(700);
  esc3.write(700);
  esc4.write(700);
  delay(2000);
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
}

