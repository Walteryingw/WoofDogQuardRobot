 
//int state=0;  //调试模式1，运行模式0

#define LF_H 1  //左前大腿
#define LF_S 0   //左前小腿
#define LFS 2  //左前肩

#define RF_H 14   //右前大腿
#define RF_S 15   //右前小腿
#define RFS 13   //右前肩
 
#define LR_H  5  //左后大腿
#define LR_S 4   //左后小腿
#define LBS 6   //左后肩

#define RR_H 10  //右后大腿
#define RR_S 11   //右后小腿
#define RBS 9   //右后肩
 
#define neck 7   //脖子
#define head 8   //头
int mode=0;//自动/手动控制
float x1,y5,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,s1,bl1,sl1,s2,bl2,sl2,s3,bl3,sl3,s4,bl4,sl4,M4,M1,M2,M3,t=0,A=0,D=0,Z,AX,DX,a,b,c,d,z=0,zz=0;// Tm为一个周期，y0，y1在ESP32ARDUINO里被定义了，改成y5
float r1=45,r2=90;
int L1=25,L2=80,L3=70;//L1单腿横向宽度，L2大腿长度，L3小腿长度
int S0=20,H0=30;//S0迈腿长度，H0迈腿高度
int Ht=100,Ht1=100,Ht2=100,Ht3=100,Ht4=100;//Hc整体高度 /// Hc0,1,2,3腿1/2/3/4高度,用于姿态
int wi =50 ;  //机器人宽度
int w=50;
int l=180; //机器人长度
/*默认速度 步长 设置*/
int ps=1; //pwm停止和恢复
float Tm = 0.5;          //周期
//float faai = 1;      //占空比
float speed = 0.025; //步频调节
int val=0;
String k="";
/*各舵机站立初始值，每个机械狗初始值不同，根据需要自行调整, 按校准键后，机器狗成标准姿态(见安装调试说明)
*                      
*     Z0：1号大腿   -----   Z1：2号大腿
*     Z4：1号小腿   -----   Z5：2号小腿
*     Z8：1号肩膀   -----   Z9：2号肩膀                    
*     
*     Z2：3号大腿   -----   Z3：4号大腿
*     Z6：3号小腿   -----   Z7：4号小腿
*    Z10：3号肩膀   -----  Z11：4号肩膀       
*/

int D1=135,D2=45,D3=135,D4=45; //大腿
int X1=90,X2=90,X3=90,X4=90; // 小腿
int J1=90,J2=90,J3=90,J4=90;// 肩膀


/*注意修改以下WIFI名称和密码！！！！！！
 * 
 */
const char* ssid     = "XXXXXXXX";   ///// WIFI
const char* password = "12345678"; ////// WIFI密码
/*
 */
