
#include "config.h"   //参数配置文件


#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Adafruit_PWMServoDriver.h>     //PCA9685驱动
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/*
  #define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
  #define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)*/
#define USMIN  500 // This is the rounded 'minimum' microsecond length  
#define USMAX  2500 // This is the rounded 'maximum' microsecond length

#include "MPU6050_6Axis_MotionApps20.h"
//MPU6050 mpu6050(Wire,0.1,0.9);
MPU6050 mpu;

unsigned long now, lastTime = 0;
float dt;                                   //微分时间
int  count = 0;
int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; //角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量
int agxx = 0, agyy = 0;
float AcceRatio = 16384.0;                  //加速度计比例系数
float GyroRatio = 131.0;                    //陀螺仪比例系数

uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum, aaz_sum;                     //x,y轴采样和

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0} , g_x[10] = {0} , g_y[10] = {0}, g_z[10] = {0}; //加速度计协方差计算队列
float Px = 1, Rx, Kx, Sx, Vx, Qx;           //x轴卡尔曼变量
float Py = 1, Ry, Ky, Sy, Vy, Qy;           //y轴卡尔曼变量
float Pz = 1, Rz, Kz, Sz, Vz, Qz;           //z轴卡尔曼变量



#include "BluetoothSerial.h"   //蓝牙模块
BluetoothSerial SerialBT;

void bth() {  //蓝牙

  SerialBT.begin("WoofogV2"); //蓝牙模块名称
  Serial.println("蓝牙已启动，你可以配对了！");
}

WiFiServer server(80);   //wifi端口

void wificon() {          //wifi连接

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  /* while (WiFi.waitForConnectResult() != WL_CONNECTED) {
     Serial.println("Connection Failed! Rebooting...");
     delay(3000);
     ESP.restart();
    } */
}

void OTA() {
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()    //初始化
{
  Serial.begin(115200);
  wificon();  //开启WIFI
  bth();      //开启蓝牙
  // fsStart(); //文件
  //OTA();  //OTA升级功能

  Wire.begin(21, 22);   //ESP32的21/22针对应SDA,SCL

  mpu.initialize();                 //初始化
  delay(500);
  /*    PTF("Initializing I2C devices...");
      PTF("Testing device connections...");

      PTF(mpu.testConnection() ? "MPU6050 con successful" : "MPU6050 con failed");
  */
  /*  mpu.setXAccelOffset(1676);
    mpu.setYAccelOffset(1337);
    mpu.setZAccelOffset(968);
    mpu.setXGyroOffset(-310);
    mpu.setYGyroOffset(-85);
    mpu.setZGyroOffset(44);
  */
  unsigned short times = 200;             //采样次数
  for (int i = 0; i < times; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
    axo += ax; ayo += ay; azo += az;      //采样和
    gxo += gx; gyo += gy; gzo += gz;
  }
  axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
  gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移

  pwm.begin();
  pwm.setPWMFreq(50);   //50HZ更新频率，相当于20ms的周期
  pwm.setOscillatorFrequency(27000000); //
  //理论上内部振荡器是25MHz，但实际上不是
  //非常精确。你可以调整这个数字直到
  //你得到了你期望的频率！

}
void calculate()//运动函数
{
  s1 = atan(-z1 / y5) - atan(L1 / sqrt(y5 * y5 + z1 * z1 - L1 * L1)); //腿1转弯角度（肩关节角度）
  s2 = atan(-z2 / y2) - atan(L1 / sqrt(y2 * y2 + z2 * z2 - L1 * L1)); //腿2转弯角度（肩关节角度）
  s3 = atan(-z3 / y3) - atan(L1 / sqrt(y3 * y3 + z3 * z3 - L1 * L1)); //腿3转弯角度（肩关节角度）
  s4 = atan(-z4 / y4) - atan(L1 / sqrt(y4 * y4 + z4 * z4 - L1 * L1)); //腿4转弯角度（肩关节角度）

  M1 = (-y5 + L1 * sin(s1)) / cos(s1);
  M2 = (-y2 + L1 * sin(s2)) / cos(s2);
  M3 = (-y3 + L1 * sin(s3)) / cos(s3);
  M4 = (-y4 + L1 * sin(s4)) / cos(s4);

  float Lx = L2 * L2 - L3 * L3;
  float Ls = L2 * L2 + L3 * L3;
  /////////////////////////////////////

  bl1 = (-asin(x1 / sqrt(x1 * x1 + M1 * M1)) + acos((x1 * x1 + M1 * M1 + Lx) / (2 * L2 * sqrt(x1 * x1 + M1 * M1)))) * 180 / PI - r1; //腿1大腿摆动角度
  bl2 = (-asin(x2 / sqrt(x2 * x2 + M2 * M2)) + acos((x2 * x2 + M2 * M2 + Lx) / (2 * L2 * sqrt(x2 * x2 + M2 * M2)))) * 180 / PI - r1; //腿2大腿摆动角度
  bl3 = (-asin(x3 / sqrt(x3 * x3 + M3 * M3)) + acos((x3 * x3 + M3 * M3 + Lx) / (2 * L2 * sqrt(x3 * x3 + M3 * M3)))) * 180 / PI - r1; //腿3大腿摆动角度
  bl4 = (-asin(x4 / sqrt(x4 * x4 + M4 * M4)) + acos((x4 * x4 + M4 * M4 + Lx) / (2 * L2 * sqrt(x4 * x4 + M4 * M4)))) * 180 / PI - r1; //腿4大腿摆动角度

  sl1 = (acos(-(x1 * x1 + M1 * M1 - Ls) / (2 * L2 * L3))) * 180 / PI - r2; //腿1小腿摆动角度
  sl2 = (acos(-(x2 * x2 + M2 * M2 - Ls) / (2 * L2 * L3))) * 180 / PI - r2; //腿2小腿摆动角度
  sl3 = (acos(-(x3 * x3 + M3 * M3 - Ls) / (2 * L2 * L3))) * 180 / PI - r2; //腿3小腿摆动角度
  sl4 = (acos(-(x4 * x4 + M4 * M4 - Ls) / (2 * L2 * L3))) * 180 / PI - r2; //腿4小腿摆动角度

  //180 * shank1 / PI;弧度转角度
  s1 = 180 * s1 / PI;
  s2 = 180 * s2 / PI;
  s3 = 180 * s3 / PI;
  s4 = 180 * s4 / PI;

}
float d2p(float deg)  //角度转换成舵机毫秒脉冲
{
  if (deg <= 180 && deg >= 0) { //保护舵机 不进入不可用角度以免损坏
    //return int(map(deg, 180, 10, SERVOMAX, SERVOMIN));
    return int(map(deg, 180, 0, USMAX, USMIN));
  }
  else if (deg > 180)
  {
    return 2500;
  }
  else if (deg < 0)
  {
    return 500;
  }
}

void servopwm()//关节计算角度对应到舵机转角, pwm.writeMicroseconds用的是毫秒,还有pwm.setPWM(),用的是脉冲
{ calculate();
  pwm.writeMicroseconds(LFS, d2p(J1 + s1)); //腿1肩关节横向转角
  pwm.writeMicroseconds(LF_H, d2p(D1 + bl1)); //腿1大腿转角
  pwm.writeMicroseconds(LF_S, d2p(X1 + sl1)); //腿1小腿转角   /////

  pwm.writeMicroseconds(RFS, d2p(J2 - s2)); //腿2肩关节横向转角
  pwm.writeMicroseconds(RF_H, d2p(D2 - bl2)); //腿2大腿转角
  pwm.writeMicroseconds(RF_S, d2p(X2 - sl2)); //腿2小腿转角     /////

  pwm.writeMicroseconds(LBS, d2p(J3 - s3)); //腿3肩关节横向转角
  pwm.writeMicroseconds(LR_H, d2p(D3 + bl3)); //腿3大腿转角        ///
  pwm.writeMicroseconds(LR_S, d2p(X3 + sl3)); //腿3小腿转角       /////

  pwm.writeMicroseconds(RBS, d2p(J4 + s4)); //腿4肩关节横向转角
  pwm.writeMicroseconds(RR_H,  d2p(D4 - bl4)); //腿4大腿转角       ////
  pwm.writeMicroseconds(RR_S,  d2p(X4 - sl4)); //腿4小腿转角                ////
}
void calib()    //调中,所有90度
{
  pwm.writeMicroseconds(LFS, d2p(J1));//腿1肩关节横向转角
  pwm.writeMicroseconds(LF_H, d2p(D1 - 45)); //腿1大腿转角
  pwm.writeMicroseconds(LF_S, d2p(X1));//腿1小腿转角

  pwm.writeMicroseconds(RFS, d2p(J2));//腿2肩关节横向转角
  pwm.writeMicroseconds(RF_H, d2p(D2 + 45)); //腿2大腿转角
  pwm.writeMicroseconds(RF_S, d2p(X2));//腿2小腿转角

  pwm.writeMicroseconds(LBS, d2p(J3));//腿3肩关节横向转角
  pwm.writeMicroseconds(LR_H, d2p(D3 - 45)); //腿3大腿转角
  pwm.writeMicroseconds(LR_S, d2p(X3));//腿3小腿转角

  pwm.writeMicroseconds(RBS, d2p(J4));//腿4肩关节横向转角
  pwm.writeMicroseconds(RR_H,  d2p(D4 + 45)); //腿4大腿转角
  pwm.writeMicroseconds(RR_S,  d2p(X4));//腿4小腿转角

}

void stand()    //站立
{
  pwm.writeMicroseconds(LFS, d2p(J1));//腿1肩关节横向转角
  pwm.writeMicroseconds(LF_H, d2p(D1));//腿1大腿转角
  pwm.writeMicroseconds(LF_S, d2p(X1));//腿1小腿转角

  pwm.writeMicroseconds(RFS, d2p(J2));//腿2肩关节横向转角
  pwm.writeMicroseconds(RF_H, d2p(D2));//腿2大腿转角
  pwm.writeMicroseconds(RF_S, d2p(X2));//腿2小腿转角

  pwm.writeMicroseconds(LBS, d2p(J3));//腿3肩关节横向转角
  pwm.writeMicroseconds(LR_H, d2p(D3));//腿3大腿转角
  pwm.writeMicroseconds(LR_S, d2p(X3));//腿3小腿转角

  pwm.writeMicroseconds(RBS, d2p(J4));//腿4肩关节横向转角
  pwm.writeMicroseconds(RR_H, d2p(D4));//腿4大腿转角
  pwm.writeMicroseconds(RR_S, d2p(X4));//腿4小腿转角
}
void rests(){
  pwm.writeMicroseconds(LFS, d2p(J1));//腿1肩关节横向转角
  pwm.writeMicroseconds(LF_H, d2p(D1+15));//腿1大腿转角
  pwm.writeMicroseconds(LF_S, d2p(X1-70));//腿1小腿转角

  pwm.writeMicroseconds(RFS, d2p(J2));//腿2肩关节横向转角
  pwm.writeMicroseconds(RF_H, d2p(D2-15));//腿2大腿转角
  pwm.writeMicroseconds(RF_S, d2p(X2+70));//腿2小腿转角

  pwm.writeMicroseconds(LBS, d2p(J3));//腿3肩关节横向转角
  pwm.writeMicroseconds(LR_H, d2p(D3+15));//腿3大腿转角
  pwm.writeMicroseconds(LR_S, d2p(X3-70));//腿3小腿转角

  pwm.writeMicroseconds(RBS, d2p(J4));//腿4肩关节横向转角
  pwm.writeMicroseconds(RR_H, d2p(D4-15));//腿4大腿转角
  pwm.writeMicroseconds(RR_S, d2p(X4+70));//腿4小腿转角
  
}
void trot()//小跑步态
{
  d = atan(z / Ht);
  r1 = 45;
  r2 = 90;
  H0 = 35; //抬腿最大高度，Y轴方向


  if (t < Tm) //前半周期，1、4腿在后；2、3腿在前
  {
    c = cos(d * cos(PI * t / Tm)); //c参数，接下来带入方程
    b = (0.5 - 0.5 * cos(2 * PI * t / Tm)); //b参数，接下来带入方程

    x1 = S0 * (t / Tm - 1 / (2 * PI) * sin(2 * PI * t / Tm)) - 0.5 * S0; //腿1 X轴坐标，前进方向，S0为迈腿长度
    y5 = (H0 * b - Ht) / c; //腿1 Y轴坐标，抬腿方向
    z1 = a + z * cos(PI * t / Tm); //腿1 Z轴坐标，转弯方向

    x4 = x1; //腿4 X轴坐标，前进方向（腿外侧为正向）  ///
    y4 = y5; //腿4 Y轴坐标
    z4 = z1; //腿4 Z轴坐标

    x2 = -x1; //腿2 X轴坐标，前进方向      ///
    y2 = (-H0 / 40 * b - Ht) / c; //腿2 Y轴坐标，抬腿方向
    //y1=-Hc/c;
    z2 = z1; //腿2 Z轴坐标，转弯方向

    x3 = x2; //腿3 X轴坐标，前进方向（腿外侧为正向）  /////
    y3 = y2; //腿3
    z3 = z1; //腿3
  }

  if (t >= Tm and t < 2 * Tm) //后半周期，2、3腿在后；1、4腿在前
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * (t - Tm) / Tm));

    x1 = 0.5 * S0 - S0 * ((t - Tm) / Tm - 1 / (2 * PI) * sin(2 * PI * (t - Tm) / Tm));
    y5 = (-H0 / 40 * b - Ht) / c;
    //y0=-Hc/c;
    z1 = a + z * cos(PI * t / Tm);

    x4 = x1; ///
    y4 = y5;
    z4 = z1;

    x2 = -x1;  ////
    y2 = (H0 * b - Ht) / c;
    z2 = z1;

    x3 = x2;
    y3 = y2;
    z3 = z1;
  }

  servopwm();
  if (t >= Tm * 2) //一个完整的运动周期结束
  {
    t = 0;
  }
}
//平移步态

void turn2()//平移步态

{
  d = atan(z / Ht);
  r1 = 45;
  r2 = 90;
  H0 = 40; //抬腿高度

  if (zz == 0) //Z轴转角，zz==0即横向无转动
  {
    H0 = 0;
  }

  if (t < Tm)
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * t / Tm));
    x1 = 0;
    y5 = (H0 * b - Ht) / c;
    z1 = a + zz * cos(PI * t / Tm);

    x2 = 0;
    //y1=(-H0/40*b-Hc)/c;   ////
    y2 = (-Ht) / c;
    z2 = a + zz * cos(PI * t / Tm);

    x3 = 0;
    y3 = y2;
    z3 = a + zz * cos(PI * t / Tm + PI);

    x4 = 0;
    y4 = y5;
    z4 = a + zz * cos(PI * t / Tm + PI);
  }

  if (t >= Tm and t < 2 * Tm)
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * (t - Tm) / Tm));
    x1 = 0;
    //y0=(-H0/40*b-Hc)/c;     ////
    y5 = -Ht / c;
    z1 = a + zz * cos(PI * t / Tm);

    x2 = 0;
    y2 = (H0 * b - Ht) / c;
    z2 = a + zz * cos(PI * t / Tm);

    x3 = 0;
    y3 = y2;
    z3 = a + zz * cos(PI * t / Tm + PI);

    x4 = 0;
    y4 = y5;
    z4 = a + zz * cos(PI * t / Tm + PI);
  }

  servopwm();

  if (t >= Tm * 2) //一个完整的平移周期
  {
    t = 0;
  }
}



void walk()//爬行步态，行走步态，相序为1-4-2-3，每个动作四分之一周期

{
  d = atan(z / Ht);
  r1 = 45;
  r2 = 90;
  H0 = 40;

  if (t < Tm) //第四分之一周期
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * t / Tm));

    x1 = S0 * (t / Tm - 1 / (2 * PI) * sin(2 * PI * (t + Tm) / Tm)) - 0.5 * S0;
    y5 = (H0 * b - Ht) / c;
    z1 = a + z * cos(PI * t / Tm);

    x3 = 0.5 * S0 - S0 * (t / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * t / (3 * Tm)));
    y3 = (-Ht) / c;
    z3 = a + z * cos(PI * t / (3 * Tm));

    x2 = 0.5 * S0 - S0 * ((t + Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t + Tm) / (3 * Tm)));
    y2 = (-Ht) / c;
    z2 = a + z * cos(PI * (t + Tm) / (3 * Tm));

    x4 = 0.5 * S0 - S0 * ((t + 2 * Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t + 2 * Tm) / (3 * Tm)));
    y4 = (-Ht) / c;
    z4 = a + z * cos(PI * (t + 5 * Tm) / (3 * Tm));

    //x4=-x4;
    //x3=-x3;
  }

  if (t >= Tm) //第四分之二周期
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * (t - Tm) / Tm));

    x1 = 0.5 * S0 - S0 * ((t - Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - Tm) / (3 * Tm)));
    y5 = (-Ht) / c;
    z1 = a + z * cos(PI * (t + 2 * Tm) / (3 * Tm));

    x3 = 0.5 * S0 - S0 * (t / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * t / (3 * Tm)));
    y3 = (-Ht) / c;
    z3 = a + z * cos(PI * t / (3 * Tm));

    x2 = 0.5 * S0 - S0 * ((t + Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t + Tm) / (3 * Tm)));
    y2 = (-Ht) / c;
    z2 = a + z * cos(PI * (t + Tm) / (3 * Tm));

    x4 = S0 * ((t - Tm) / Tm - 1 / (2 * PI) * sin(2 * PI * (t) / Tm)) - 0.5 * S0;
    y4 = (H0 * b - Ht) / c;
    z4 = a + z * cos(PI * (t - Tm) / Tm);

    //x4=-x4;
    //x3=-x3;
  }

  if (t >= 2 * Tm) //第四分之三周期
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * (t - 2 * Tm) / Tm));

    x1 = 0.5 * S0 - S0 * ((t - Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - Tm) / (3 * Tm)));
    y5 = (-Ht) / c;
    z1 = a + z * cos(PI * (t + 2 * Tm) / (3 * Tm));

    x3 = 0.5 * S0 - S0 * (t / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * t / (3 * Tm)));
    y3 = (-Ht) / c;
    z3 = a + z * cos(PI * t / (3 * Tm));

    x2 = S0 * ((t - 2 * Tm) / Tm - 1 / (2 * PI) * sin(2 * PI * (t - Tm) / Tm)) - 0.5 * S0;
    y2 = (H0 * b - Ht) / c;
    z2 = a + z * cos(PI * (t - Tm) / Tm);

    x4 = 0.5 * S0 - S0 * ((t - 2 * Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - 2 * Tm) / (3 * Tm)));
    y4 = (-Ht) / c;
    z4 = a + z * cos(PI * (t + Tm) / (3 * Tm));

    //x4=-x4;
    //x3=-x3;

  }

  if (t >= 3 * Tm) //第四分之四周期
  {
    c = cos(d * cos(PI * t / Tm));
    b = (0.5 - 0.5 * cos(2 * PI * (t - 3 * Tm) / Tm));

    x1 = 0.5 * S0 - S0 * ((t - Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - Tm) / (3 * Tm)));
    y5 = (-Ht) / c;
    z1 = a + z * cos(PI * (t + 2 * Tm) / (3 * Tm));

    x3 = S0 * ((t - 3 * Tm) / Tm - 1 / (2 * PI) * sin(2 * PI * (t - 2 * Tm) / Tm)) - 0.5 * S0;
    y3 = (H0 * b - Ht) / c;
    z3 = a + z * cos(PI * (t - 2 * Tm) / Tm);

    x2 = 0.5 * S0 - S0 * ((t - 3 * Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - 3 * Tm) / (3 * Tm)));
    y2 = (-Ht) / c;
    z2 = a + z * cos(PI * (t - 3 * Tm) / (3 * Tm));

    x4 = 0.5 * S0 - S0 * ((t - 2 * Tm) / (3 * Tm) - 1 / (2 * PI) * sin(2 * PI * (t - 2 * Tm) / (3 * Tm)));
    y4 = (-Ht) / c;
    z4 = a + z * cos(PI * (t + Tm) / (3 * Tm));

    //x4=-x4;
    //x3=-x3;
  }
  servopwm();

  if (t >= Tm * 4)
  {
    t = 0;
  }

}

void mpu6050()//自稳模块
{

  unsigned long now = millis();             //当前时间(ms)
  dt = (now - lastTime) / 1000.0;           //微分时间(s)
  lastTime = now;                           //上一次采样时间(ms)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
  float accx = ax / AcceRatio;              //x轴加速度
  float accy = ay / AcceRatio;              //y轴加速度
  float accz = az / AcceRatio;              //z轴加速度
  aax = atan(accy / accz) * (-180) / PI;    //y轴对于z轴的夹角
  aay = atan(accx / accz) * 180 / PI;       //x轴对于z轴的夹角
  aaz = atan(accz / accy) * 180 / PI;       //z轴对于y轴的夹角

  aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
  aay_sum = 0;
  aaz_sum = 0;

  for (int i = 1; i < n_sample; i++)
  {
    aaxs[i - 1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i - 1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i - 1] = aazs[i];
    aaz_sum += aazs[i] * i;
  }

  aaxs[n_sample - 1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0; //角度调幅至0-90°
  aays[n_sample - 1] = aay;                      //此处应用实验法取得合适的系数
  aay_sum += aay * n_sample;                     //本例系数为9/7
  aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aazs[n_sample - 1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;

  float gyrox = - (gx - gxo) / GyroRatio * dt; //x轴角速度
  float gyroy = - (gy - gyo) / GyroRatio * dt; //y轴角速度
  float gyroz = - (gz - gzo) / GyroRatio * dt; //z轴角速度
  agx += gyrox;                             //x轴角速度积分
  agy += gyroy;                             //x轴角速度积分
  agz += gyroz;

  /* kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;

  for (int i = 1; i < 10; i++)
  { //测量值平均值运算
    a_x[i - 1] = a_x[i]; //即加速度平均值
    Sx += a_x[i];
    a_y[i - 1] = a_y[i];
    Sy += a_y[i];
    a_z[i - 1] = a_z[i];
    Sz += a_z[i];
  }

  a_x[9] = aax;
  Sx += aax;
  Sx /= 10;   //x轴加速度平均值
  a_y[9] = aay;
  Sy += aay;
  Sy /= 10;   //y轴加速度平均值
  a_z[9] = aaz;
  Sz += aaz;
  Sz /= 10;

  for (int i = 0; i < 10; i++)
  {
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);
  }



  Rx = Rx / 9;      //得到方差
  Ry = Ry / 9;
  Rz = Rz / 9;

  Px = Px + 0.0025;     // 0.0025在下面有说明...
  Kx = Px / (Px + Rx);      //计算卡尔曼增益
  agx = agx + Kx * (aax - agx);     //陀螺仪角度与加速度计速度叠加
  Px = (1 - Kx) * Px;                       //更新p值

  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz);
  Pz = (1 - Kz) * Pz;

  /* kalman end */

  D = -agx / 180 * PI; //修正  /////////////  此处根据MPU实际安装的方向需做调整
  A = -agy / 180 * PI; //修正   /////////////
  //  z=-agz/180*PI; //

}


void stable()//静态方程
{
  float Z;
  Z = z / 180.0 * PI; //E为航向角

  x1 = -Ht1 * cos(A) * sin(A) + (l / 2 * cos(0.2 * PI - Z) - w); //a为俯仰角//
  y5 = -Ht1 * cos(A) * cos(A) / cos(D); //d为横滚角
  z1 = - Ht1 * D + (l / 2 * sin(0.2 * PI - Z) - w / 2);

  x2 = -Ht2 * cos(A) * sin(A) + (l / 2 * cos(0.2 * PI + Z) - w); //a为俯仰角
  y2 = -Ht2 * cos(A) * cos(A) / cos(D); //d为横滚角
  z2 = Ht2 * D + l / 2 * sin(0.2 * PI + Z) - w / 2;

  x3 = -Ht3 * cos(A) * sin(A);
  // x3 = -Hc2 * cos(A) * sin(A) + (1 / 2 * cos(0.2 * PI + Z)-w/2);
  y3 = -Ht3 * cos(A) * cos(A) / cos(D);
  z3 = -Ht3 * D + (l / 2 * sin(0.2 * PI + Z) - w / 2);

  x4 = -Ht4 * cos(A) * sin(A) - (l / 2 * cos(0.2 * PI - Z) - w);
  y4 = -Ht4 * cos(A) * cos(A) / cos(D);
  z4 = Ht4* D + (l / 2 * sin(0.2 * PI - Z) - w / 2);

  x1 = -x1;
  x2 = -x2;

  servopwm();

}



void balance()//自稳
{
  mpu6050(); //调用MPU角度
  //   Hc=100;
  //   A=-A;     //+/- 根据MPU的方向来定
  //  D=-D;     //+/- 根据MPU的方向来定

  c = Ht - (w / 2) * sin(D); 
  b = Ht - (l / 2) * sin(A); 
  Ht3 = c + b - Ht; //腿3高度
  Ht2 = 2 * Ht - Ht3; //腿2高度
  Ht1 = 2 * c - Ht3; //腿1高度
  Ht4 = 2 * Ht - Ht1; //腿4高度
  D = -cos(D) * sin(D);
  stable();

}
void zitai()//用于姿态
{ A = -AX / 180 * PI;
  D = DX / 180 * PI;

  c = Ht - (w / 2) * sin(D); 
  b = Ht - (l / 2) * sin(A); 
  Ht3 = c + b - Ht; //腿3高度
  Ht2 = 2 * Ht - Ht3; //腿2高度
  Ht1 = 2 * c - Ht3; //腿1高度
  Ht4 = 2 * Ht - Ht1; //腿4高度
  D = cos(D) * sin(D);
  stable();
}
void balance2()//用于平衡
{
  mpu6050(); //调用MPU角度
  //   A=-A;     //+/- 根据MPU的方向来定
  //  D=-D;     //+/- 根据MPU的方向来定
  c = Ht - (w / 2) * sin(D); 
  b = Ht - (l / 2) * sin(A);
  Ht3 = c + b - Ht; //腿3高度
  Ht2 = 2 * Ht - Ht3; //腿2高度
  Ht1 = 2 * c - Ht3; //腿1高度
  Ht4 = 2 * Ht - Ht1; //腿4高度
  D = cos(D) * sin(D);   // +/- 根据MPU的方向来定

  stable();

}

void baidong2()//调整高度方程，四条腿高度相同
{
  x1 = 0;
  y5 = -Ht;
  z1 = 25;

  x2 = 0;
  y2 = -Ht;
  z2 = 25;

  x3 = 0;
  y3 = -Ht;
  z3 = 25;

  x4 = 0;
  y4 = -Ht;
  z4 = 25;

  servopwm();
}

void jump() {

}

int  Distance() //超声波测距
{ /*
     digitalWrite(TrigPin,LOW);
     delayMicroseconds(2);
     digitalWrite(TrigPin,HIGH);
     delayMicroseconds(10);
     digitalWrite(TrigPin,LOW);
     int cm=pulseIn(EchoPin,HIGH)/58;
     return cm;
  */
}

#include "control.h";

void loop() {

  control();   //串口接收指令


}
