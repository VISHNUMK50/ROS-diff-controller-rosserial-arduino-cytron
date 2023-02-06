#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


#define l_dir 13 
#define r_dir 12
#define l_pwm 11
#define r_pwm 10

float left_vel=0;
float right_vel=0;
int motor_rpm = 200;            //   max rpm of motor on full voltage 
float wheel_diameter = 0.07;    //  in meters
float wheel_separation = 0.26;   //  in meters
float max_pwm_val = 255;         //  100 for Raspberry Pi , 255 for Arduino
float min_pwm_val = 0;           // Minimum PWM value that is needed for the robot to move

float wheel_radius = wheel_diameter/2;
float circumference_of_wheel = 2 * 3.14 * wheel_radius;
float max_speed = (circumference_of_wheel*motor_rpm)/60 ;  //   m/sec
std_msgs::Float32 left_wheel_tick_count;

std_msgs::Float32 right_wheel_tick_count;

void onTwist(const geometry_msgs::Twist& msg)
{
    float wheel_radius;
    float wheel_separation;

    float linear_vel = msg.linear.x ;                 //# Linear Velocity of Robot
    float angular_vel = msg.angular.z ;               //# Angular Velocity of Robot
    //print(str(linear)+"\t"+str(angular))
   
    float VrplusVl  = 2 * linear_vel;
    float VrminusVl = angular_vel * 0.26;
    
    float right_vel = ( VrplusVl + VrminusVl ) / 2;   // # right wheel velocity along the ground
    float left_vel  = VrplusVl - right_vel;  // # left wheel velocity along the ground
     left_wheel_tick_count.data = left_vel;
     right_wheel_tick_count.data = right_vel ;
    //print (str(left_vel)+"\t"+str(right_vel))
    
    if (left_vel == 0.0 and right_vel == 0.0){
        stops();
    }
    else{
        wheel_vel_executer(left_vel, right_vel);
    }
} 
  /////////////////////////////////////////////  
  
void wheel_vel_executer(float left_speed, float right_speed){
    
    
    float rspeedPWM = max(min(((abs(left_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val);
    float lspeedPWM = max(min(((abs(right_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val);
    analogWrite(l_pwm,lspeedPWM);
    analogWrite(r_pwm,rspeedPWM);
    
    
    if (left_speed >= 0){
        digitalWrite(l_dir,HIGH);
    }
    else{
        digitalWrite(l_dir,LOW);
    }
        
    if (right_speed >= 0){
        digitalWrite(r_dir,HIGH);
    }
    else{
        digitalWrite(r_dir,LOW);
    }


}
  ///////////////////////////////////
  

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); 

ros::NodeHandle nh;

ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

void setup() {
  // put your setup code here, to run once:
  pinMode(l_pwm,OUTPUT);
  pinMode(r_pwm,OUTPUT);
  pinMode(l_dir,OUTPUT);
  pinMode(r_dir,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  analogWrite(l_pwm,0);
  analogWrite(r_pwm,0);
  digitalWrite(l_dir,HIGH);
  digitalWrite(r_dir,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  rightPub.publish( &right_wheel_tick_count );
   leftPub.publish( &left_wheel_tick_count );
  nh.spinOnce();
  
}


void stops(){
  analogWrite(l_pwm,0);
  analogWrite(r_pwm,0);
  digitalWrite(l_dir,HIGH);
  digitalWrite(r_dir,HIGH);
}
