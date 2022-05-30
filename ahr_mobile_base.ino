
#include <PID_v1.h>

#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <mobile_robot_test/custom_odom.h>

#include <mobile_robot_test/custom_cmd.h>

//left encoder
#define left_encodPinA      3                      // Quadrature encoder A pin
#define left_encodPinB      9                       // Quadrature encoder B pin


//left motor
#define left_M1              7                       // PWM outputs to motor driver module
#define left_M2              6

//right encoder
#define right_encodPinA      2                       // Quadrature encoder A pin
#define right_encodPinB      8                       // Quadrature encoder B pin

//right motor
#define right_M1             4                       // PWM outputs to motor driver module
#define right_M2             5



double kp_l =0.01, ki_l =0.5 , kd_l =0;             // modify for optimal performance
double kp_r =0.01, ki_r =0.5 , kd_r =0;             // modify for optimal performance


double input_left = 0, output_left = 0, setpoint_left = 0;
double input_right = 0, output_right = 0, setpoint_right = 0;


PID left_PID(&input_left, &output_left, &setpoint_left, kp_l, ki_l, kd_l,DIRECT);  
PID right_PID(&input_right, &output_right, &setpoint_right, kp_r, ki_r, kd_r,DIRECT);  

ros::NodeHandle  nh;

unsigned long lastTime,now,lasttimepub;

volatile long encoderPos_left = 0,last_pos_left=0;
volatile long encoderPos_right = 0,last_pos_right=0;


double pos_left = 0, vel_left= 0;
double pos_right = 0, vel_right= 0;

mobile_robot_test::custom_odom enc_msg;

void set_angle_cb( const mobile_robot_test::custom_cmd& cmd_msg){
  setpoint_right= cmd_msg.r_vel_cmd;
  setpoint_left= cmd_msg.l_vel_cmd; 
}

ros::Subscriber<mobile_robot_test::custom_cmd> sub("/joints_to_aurdino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &enc_msg);


void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  pinMode(left_encodPinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(left_encodPinB, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(left_encodPinA), encoder_left, FALLING);               // update encoder position

  pinMode(right_encodPinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(right_encodPinB, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(right_encodPinA), encoder_right, FALLING);               // update encoder position

  left_PID.SetMode(AUTOMATIC);
  left_PID.SetSampleTime(1);
  left_PID.SetOutputLimits(-255, 255);

  right_PID.SetMode(AUTOMATIC);
  right_PID.SetSampleTime(1);
  right_PID.SetOutputLimits(-255, 255);
  

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise  
//  Serial.begin(9600);
//  Serial.println("serial started");
}

void loop() {

  pos_left = (encoderPos_left*360)/9031 ;
  pos_right = (encoderPos_right*360)/9031 ;
  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=500 )
  {
      vel_left = (360.0*1000*(encoderPos_left-last_pos_left)) /(9031.0*(now - lastTime));
      vel_right = (360.0*1000*(encoderPos_right-last_pos_right)) /(9031.0*(now - lastTime));
      input_left = vel_left; 
      input_right = vel_right;
      
      
      lastTime=now;
      last_pos_left=encoderPos_left;
      last_pos_right=encoderPos_right;
  }
  
  left_PID.Compute();                                    
  pwmOut_left(output_left);

  right_PID.Compute();                                   
  pwmOut_right(output_right);

    if ((now - lasttimepub)> 100)
  {
    enc_msg.l_pose=pos_left;
    enc_msg.l_vel=vel_left;
    enc_msg.r_pose=pos_right;
    enc_msg.r_vel=vel_right;
    pub.publish(&enc_msg);
    lasttimepub=now;
  }

  nh.spinOnce();


//  Serial.print("Setpoint_right");
//  Serial.print("        ");
//  Serial.print(setpoint_right);
//
//  Serial.print("        ");
//  Serial.print("Input_right");
//  Serial.print("        ");
//  Serial.print(input_right);
//
//  Serial.print("        ");
//  Serial.print("Output_right");
//  Serial.print("        ");
//  Serial.println(output_right);
//
//  Serial.print("Setpoint_left");
//  Serial.print("        ");
//  Serial.print(setpoint_left);
//
//  Serial.print("        ");
//  Serial.print("Input_left");
//  Serial.print("        ");
//  Serial.print(input_left);
//
//  Serial.print("        ");
//  Serial.print("Output_left");
//  Serial.print("        ");
//  Serial.println(output_left);

}


void encoder_left()  {                                     // pulse and direction, direct port reading to save cycles
  if(digitalRead(left_encodPinB)==HIGH)   encoderPos_left++;
  if(digitalRead(left_encodPinB)==LOW)   encoderPos_left--;
}

void encoder_right()  {                                     // pulse and direction, direct port reading to save cycles
  if(digitalRead(right_encodPinB)==HIGH)   encoderPos_right--;
  if(digitalRead(right_encodPinB)==LOW)   encoderPos_right++;
}


void pwmOut_left(float out_left) {                                
  if (out_left > 0) {
    analogWrite(left_M2, out_left);                             // drive motor CW
    analogWrite(left_M1, 0);
  }
  else {
    analogWrite(left_M2, 0);
    analogWrite(left_M1, abs(out_left));                        // drive motor CCW
  }
}


void pwmOut_right(float out_right) {                                
  if (out_right > 0) {
    analogWrite(right_M2, out_right);                             // drive motor CW
    analogWrite(right_M1, 0);
  }
  else {
    analogWrite(right_M2, 0);
    analogWrite(right_M1, abs(out_right));                        // drive motor CCW
  }
}
