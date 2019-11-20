#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <quadrature.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include "HCPCA9685.h"
#define I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);




int start_link_0 = 195;
int start_link_1 = 27;
int start_link_2 = 345;
int start_link_3 = 225;
int start_link_4 = 18;
int start_link_5 = 210;
int start_gripper = 300;



// Initialize PID paramaters

double Setpoint_fl, Input_fl, Output_fl;
double Setpoint_fr, Input_fr, Output_fr;
double Setpoint_bl, Input_bl, Output_bl;
double Setpoint_br, Input_br, Output_br;

double aggKp=750, aggKi=4300, aggKd=0.25;


PID myPID_fl(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_fr(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT);
PID myPID_bl(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_br(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT);

// Initialize quadrature encoder paramaters

Quadrature_encoder<47,46> encoder_fright(Board::due);
Quadrature_encoder<48,49> encoder_fleft(Board::due);
Quadrature_encoder<42,43> encoder_bright(Board::due);
Quadrature_encoder<44,45> encoder_bleft(Board::due);

// Initialize pin numbers

const uint8_t RF_PWM = 11;
const uint8_t RF_BACK = 26;
const uint8_t RF_FORW = 27;
const uint8_t LF_BACK = 24;
const uint8_t LF_FORW = 25;
const uint8_t LF_PWM = 4;

const uint8_t RB_PWM = 12;
const uint8_t RB_BACK = 31;
const uint8_t RB_FORW = 30;
const uint8_t LB_BACK = 28;
const uint8_t LB_FORW = 29;
const uint8_t LB_PWM = 3;


bool wtf;



// Initialize ROS paramaters

ros::NodeHandle nh;


std_msgs::Int32 lfcount;
std_msgs::Int32 rfcount;
std_msgs::Int32 lbcount;
std_msgs::Int32 rbcount;

ros::Publisher lfwheel("lfwheel", &lfcount);
ros::Publisher rfwheel("rfwheel", &rfcount);
ros::Publisher lbwheel("lbwheel", &lbcount);
ros::Publisher rbwheel("rbwheel", &rbcount);


std_msgs::Float32 lfvel;
std_msgs::Float32 rfvel;
std_msgs::Float32 lbvel;
std_msgs::Float32 rbvel;


ros::Publisher lfspeed("lfspeed", &lfvel);
ros::Publisher rfspeed("rfspeed", &rfvel);
ros::Publisher lbspeed("lbspeed", &lbvel);
ros::Publisher rbspeed("rbspeed", &rbvel);

//servo callback

void servo_cb(const std_msgs::Int16MultiArray& cmd_msg){

  HCPCA9685.Servo(2, cmd_msg.data[0]);
  HCPCA9685.Servo(0, cmd_msg.data[1]);
  HCPCA9685.Servo(1, cmd_msg.data[2]);
  HCPCA9685.Servo(3, cmd_msg.data[3]);
  HCPCA9685.Servo(7, cmd_msg.data[4]);
  HCPCA9685.Servo(9, cmd_msg.data[5]);
  
  }
//gripper callback

void gripper_cb(const std_msgs::Int16& cmd_msg){

  HCPCA9685.Servo(10, cmd_msg.data);
  
  }

//set pid callback

void onKp(const std_msgs::Int32 &msg)
{
    aggKp = msg.data;
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
}
void onKi(const std_msgs::Int32 &msg)
{
    aggKi = msg.data;
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
}
void onKd(const std_msgs::Int32 &msg)
{
    aggKd = msg.data;
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
}

// Cmd_vel Callback
// Sets the setpoints of the pid for each wheel

//const float wheel_radius = 0.04;
//const float wheel_seperation_width = 0.1375;
//const float wheel_seperation_length = 0.13;

void onTwist(const geometry_msgs::Twist &msg)
{

  float x = msg.linear.x;
  float z = msg.angular.z;
  float w = 0.25;
  if( x > 0.25) {x = 0.25;}
  if( x < -0.25){x = -0.25;}  
  if( z > 0.2) {z = 0.2;}
  if(z < -0.2){z = -0.2;}


  if(x==0 && z==0){wtf = true;}
  else {wtf=false;}


  Setpoint_fr = x + (z * w / 2.0)/0.1;
  Setpoint_fl = x - (z * w / 2.0)/0.1;
  Setpoint_br = x + (z * w / 2.0)/0.1;
  Setpoint_bl = x - (z * w / 2.0)/0.1;
  
  //Setpoint_fl = (x - y - z*(wheel_seperation_width + wheel_seperation_length)); //int( -32 * (1.0/wheel_radius) * 
  //Setpoint_fr = (x + y + z*(wheel_seperation_width + wheel_seperation_length));
  //Setpoint_bl = (x + y - z*(wheel_seperation_width + wheel_seperation_length));
  //Setpoint_br = (x - y + z*(wheel_seperation_width + wheel_seperation_length));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);
ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo_cmd", &servo_cb);
ros::Subscriber<std_msgs::Int16> gripper_sub("gripper_cmd", &gripper_cb);
ros::Subscriber<std_msgs::Int32> Kp_sub("Kp_set", &onKp);
ros::Subscriber<std_msgs::Int32> Ki_sub("Ki_set", &onKi);
ros::Subscriber<std_msgs::Int32> Kd_sub("Kd_set", &onKd);
// Move any motor function with speed_pwm value and pin numbers

void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}



// Initialize pins for forward movement

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
}

// Encoders tend to reverse regarding of the pins??
// This way we move the robot forward a bit on startup
// And if an encoder has negative value we reverse it.

void fix_encoder_ori_on_start(){

  analogWrite(RF_PWM, 200);
  analogWrite(LF_PWM, 200);
  analogWrite(RB_PWM, 200);  ///200
  analogWrite(LB_PWM, 200); //200
  
  delay(350);

  analogWrite(RF_PWM, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RB_PWM, 0);
  analogWrite(LB_PWM, 0);

  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  if(ct1 < 0) {encoder_fleft.reverse();}
  if(ct2 < 0) {encoder_fright.reverse();}
  if(ct3 < 0) {encoder_bleft.reverse();}
  if(ct4 < 0) {encoder_bright.reverse();}
  
}

//void reset Integral error when we stop
void reset_pid_Ki()
{
  myPID_fl.SetMode(MANUAL);
  myPID_fr.SetMode(MANUAL);
  myPID_bl.SetMode(MANUAL);
  myPID_br.SetMode(MANUAL);
  Output_fl=0;
  Output_fr=0;
  Output_bl=0;
  Output_br=0;

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

}



void setup() {

  // 115200 baud rate
  nh.getHardware()->setBaud(115200);

  // Pid setup
  
  myPID_fl.SetOutputLimits(-255, 255);
  myPID_fr.SetOutputLimits(-255, 255);
  myPID_bl.SetOutputLimits(-255, 255);
  myPID_br.SetOutputLimits(-255, 255);

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

  myPID_fl.SetSampleTime(25);
  myPID_fr.SetSampleTime(25);
  myPID_bl.SetSampleTime(25);
  myPID_br.SetSampleTime(25);

  // Encoder setup
  
  encoder_fright.begin();
  encoder_fleft.begin();
  encoder_bright.begin();
  encoder_bleft.begin();
  
  // setup pins and fix encoders
   
  setpins();
  fix_encoder_ori_on_start();
  bool use_arm = false;
  if(use_arm){
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  HCPCA9685.Servo(0, start_link_1);
  HCPCA9685.Servo(1, start_link_2);
  HCPCA9685.Servo(7, start_link_4);
  HCPCA9685.Servo(2, start_link_0);
  HCPCA9685.Servo(3, start_link_3);
  HCPCA9685.Servo(9, start_link_5);
  HCPCA9685.Servo(10, start_gripper);}
  

 
    

  // ros node setup
  
  nh.initNode();
  nh.advertise(lfwheel);
  nh.advertise(rfwheel);
  nh.advertise(lbwheel);
  nh.advertise(rbwheel);
  nh.advertise(lfspeed);
  nh.advertise(rfspeed);
  nh.advertise(lbspeed);
  nh.advertise(rbspeed);

  nh.subscribe(sub);
  nh.subscribe(servo_sub);
  nh.subscribe(Kp_sub);
  nh.subscribe(Ki_sub);
  nh.subscribe(Kd_sub);
  nh.subscribe(gripper_sub);
  
}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
int old_ct1=0;
int old_ct2=0;
int old_ct3=0;
int old_ct4=0;

float ticks_per_meter = 33000.1;

void loop() {
  
  // count encoder ticks
  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  // for some reason if i omit this it does not work properly
  if (ct1!=-1){
    lfcount.data = ct1;}
  if (ct2!=-1){
    rfcount.data = ct2;}
  if (ct3!=-1){
    lbcount.data = ct3;}
  if (ct4!=-1){
    rbcount.data = ct4;}
  // Publish encoder ticks to calculate odom on Jetson Nano side
  lfwheel.publish(&lfcount);
  rfwheel.publish(&rfcount);
  lbwheel.publish(&lbcount);
  rbwheel.publish(&rbcount);


 // calculate time and current velocity
  
  unsigned long now = millis();
  Input_fl = (float(ct1 - old_ct1) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_fr = (float(ct2 - old_ct2) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_bl = (float(ct3 - old_ct3) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_br = (float(ct4 - old_ct4) / ticks_per_meter) / ((now - prev) / 1000.0);

  lfvel.data = Input_fl;
  rfvel.data = Input_fr;
  lbvel.data = Input_bl;
  rbvel.data = Input_br;

  lfspeed.publish(&lfvel);
  rfspeed.publish(&rfvel);
  lbspeed.publish(&lbvel);
  rbspeed.publish(&rbvel);


  // Compute  Pid
  myPID_fl.Compute();
  myPID_fr.Compute();
  myPID_bl.Compute();
  myPID_br.Compute();


  if(wtf){
    reset_pid_Ki();  
  }

  // Move the motors with the output of the pid
  
  Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
  Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
  Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
  Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);

  // spin the ros node
  
  nh.spinOnce();
  // take the old encoder ticks and time for calculating velocity
  old_ct1 = encoder_fleft.count();
  old_ct2 = encoder_fright.count();
  old_ct3 = encoder_bleft.count();
  old_ct4 = encoder_bright.count();

  prev = now;
  
  delay(25);

}
