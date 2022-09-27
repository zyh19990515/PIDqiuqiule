#include <Arduino.h>
#include <Servo.h>
#define ZERO_X 133
#define ZERO_Y 100

#define YL        5    
#define YH        6  
#define XL        7  
#define XH        8
Servo myservo1,myservo2;
int Flag_Stop=1,Flag_Show,Flag_Move;  //相关标志位
float Zero_X=133,Zero_Y=100,Target_X,Target_Y;  //X Y方向的目标值和控制量
int Position_X,Position_Y; //X Y方向的测量值
//float PS2_KEY;   //PS2按键变量
float Balance_Kp=53,Balance_Kd=58;//PID参数
//int PS2_LX=128,PS2_LY=128,PS2_RX=128,PS2_RY=128;     //PS2遥控相关
//int error = 0;   //PS2使用的一个变量 识别PS2是否插入
void(*resetFunc)(void)=0;
/**************************************************************************
函数功能：特定轨迹运动
入口参数：无
返回  值：无
**************************************************************************/
void Setting_Move(void)
{
     static float count;  //计数变量
     count++;  //自加
    if(Flag_Move==1)  //控制小球沿着三角形的轨迹运动
    {
             if(count<40) Zero_Y++;   
        else if(count<80) Zero_Y-=2,Zero_X-=1.5;
        else if(count<120)Zero_X+=3;
        else if(count<160)Zero_Y+=1,Zero_X-=1.5,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else if(Flag_Move==2)   //控制小球沿着球的轨迹运动
    {        
             if(count<40) Zero_Y++;
        else if(count<40+PI*40)Zero_Y=ZERO_Y+40*cos((count-40)/20),Zero_X=ZERO_X+40*sin((count-40)/20);
        else if(count<210)Zero_Y--,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else  if(Flag_Move==4)   //控制小球沿着叉(X)的轨迹运动
    {
             if(count<40) Zero_Y++,Zero_X--;
        else if(count<120)Zero_Y--,Zero_X++;
        else if(count<160)Zero_Y++,Zero_X--;
        else if(count<200)Zero_Y++,Zero_X++;
        else if(count<280)Zero_Y--,Zero_X--;
        else if(count<320)Zero_Y++,Zero_X++,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else  if(Flag_Move==8) //控制小球沿着正方形的轨迹运动
    {
             if(count<40) Zero_Y++;
        else if(count<80) Zero_X++;
        else if(count<160)Zero_Y--;
        else if(count<240)Zero_X--;
        else if(count<320)Zero_Y++;
        else if(count<360)Zero_X++;
        else if(count<400)Zero_Y--,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
}

/**************************************************************************
函数功能：PID参数调节
入口参数：无
返回  值：无
**************************************************************************/
/*
void Adjust(void)
{
    int X_temp,Y_temp,Threshold=100;//阈值
    //X_temp=PS2_RX-128;   //X方向偏差临时变量更新
    //Y_temp=PS2_RY-128;   //X方向偏差临时变量更新
    if(PS2_KEY==16||PS2_KEY==32||PS2_KEY==64||PS2_KEY==128)//左边的任意4个按键按下才能调节PID参数，防止误触
    {
    if(X_temp>Threshold) Balance_Kp++;   //KP参数增加
    if(X_temp<-Threshold)Balance_Kp--;   //KP参数减小
    if(Y_temp>Threshold) Balance_Kd--;   //KD参数减小
    if(Y_temp<-Threshold)Balance_Kd++;   //KP参数增加
    }
}
*/
/**************************************************************************
函数功能：舵机控制程序
入口参数：舵机控制量
返回  值：无
**************************************************************************/
 void Control_servo(float velocity_x,float velocity_y)
{
    myservo1.write(90-velocity_x);        // 指定舵机转向的角度
    myservo2.write(90-velocity_y);        // 指定舵机转向的角度
}

/**************************************************************************
函数功能：X方向平衡PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balanceX(float Angle )
{  
   float  Differential,Bias,Balance_Ki=0.06;//定义差分变量和偏差
   static float Last_Bias,Integration,Balance_Integration,Flag_Target;  //上一次的偏差值
   int balance;//平衡的返回值
   Bias=(Angle-Zero_X);  //===求出平衡的角度中值 和机械相关  
   Differential=Bias-Last_Bias;  //求得偏差的变化率  
  if(++Flag_Target>20) //错频处理积分控制
  {
   Flag_Target=0;
   if(Flag_Stop==0) Integration+=Bias;  // 检测到小球且舵机使能则积分
   else Integration=0;//否则清零
   if(Integration<-200) Integration=-200; //积分限幅
   if(Integration>200)  Integration=200;  
   Balance_Integration=Integration*Balance_Ki;  //积分控制
  }   
   balance=Balance_Kp*Bias/500+Balance_Kd*Differential/50+Balance_Integration;   //===计算平衡控制的舵机PWM  PD控制   kp是P系数 kd是D系数 
   Last_Bias=Bias;  //保存上一次的偏差
   return balance;  //返回值
}
/**************************************************************************
函数功能：Y方向平衡PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balanceY(float Angle )
{  
 float  Differential,Bias,Balance_Ki=0.06;//定义差分变量和偏差
   static float Last_Bias,Integration,Balance_Integration,Flag_Target;  //上一次的偏差值
   int balance;//平衡的返回值
   Bias=(Angle-Zero_Y);  //===求出平衡的角度中值 和机械相关  
   Differential=Bias-Last_Bias;  //求得偏差的变化率  
  if(++Flag_Target>20) //错频处理积分控制
  {
   Flag_Target=0;
   if(Flag_Stop==0) Integration+=Bias;  // 检测到小球且舵机使能则积分
   else Integration=0;//否则清零
   if(Integration<-200) Integration=-200; //积分限幅
   if(Integration>200)  Integration=200;  
   Balance_Integration=Integration*Balance_Ki;  //积分控制
  }   
   balance=Balance_Kp*Bias/500+Balance_Kd*Differential/50+Balance_Integration;   //===计算平衡控制的舵机PWM  PD控制   kp是P系数 kd是D系数 
   Last_Bias=Bias;  //保存上一次的偏差
   return balance;  //返回值
}

/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control(){  
  char Key;     //按键变量
  static uint8_t Flag_Target,Max_Target=30;
    
   
  digitalWrite(YL, LOW);  //给X方向+3.3V电压
  digitalWrite(YH, HIGH);   
  digitalWrite(XL, HIGH);  
  digitalWrite(XH, LOW); 
  Position_Y=analogRead(3)/5; //测量Y方向的坐标          
  digitalWrite(YL, HIGH);  
  digitalWrite(YH, LOW); 
  digitalWrite(XL, LOW);  
  digitalWrite(XH, HIGH); 
  delay(25);
  digitalWrite(YL, HIGH);  //给Y方向+3.3V电压//D3
  digitalWrite(YH, LOW); //  A0
  digitalWrite(XL, LOW);  //  D2
  digitalWrite(XH, HIGH); //  A1
  Position_X= analogRead(2)*4/15; //测量X方向的           
  digitalWrite(YL, LOW);  //
  digitalWrite(YH, HIGH); //  A0
  digitalWrite(XL, HIGH);  //  D2
  digitalWrite(XH, LOW); //  A1
    
  Target_X=-balanceX(Position_X);   //X方向的PID控制器
  Target_Y=-balanceY(Position_Y);   //Y方向的PID控制器
  if(Target_X<-Max_Target) Target_X=-Max_Target;  //X方向的舵机的控制最大角度
  if(Target_X>Max_Target)  Target_X=Max_Target;   //X方向的舵机的控制最大角度
  if(Target_Y<-Max_Target) Target_Y=-Max_Target;  //Y方向的舵机的控制最大角度
  if(Target_Y>Max_Target)  Target_Y=Max_Target;   //Y方向的舵机的控制最大角度
  if(Flag_Stop==0)Control_servo(Target_X,Target_Y); //不存在异常，控制舵机
            
 }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(128000);
  
  while (!Serial)
  {
    delay(200);
  }
  
  pinMode(XL, OUTPUT);          //电机控制引脚
  pinMode(XH, OUTPUT);          //电机控制引脚，
  pinMode(YL, OUTPUT);          //电机速度控制引脚
  pinMode(YH, OUTPUT);          //电机速度控制引脚
  myservo1.attach(10);           //初始化各个舵机
  myservo2.attach(9);            //初始化各个舵机
  myservo1.write(94);
  myservo2.write(88);
  Serial.println("setup finished");
  for(int i=0;i<5;i++){
    Serial.println(i);
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  control();
  Serial.print("kp:");
  Serial.print(Balance_Kp);
  Serial.print("    kd:");
  Serial.print(Balance_Kd);
  Serial.print("    PositionX:");
  Serial.print(Position_X);
  Serial.print("    PositionY");
  Serial.println(Position_Y);
  
}