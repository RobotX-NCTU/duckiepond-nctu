#include<ros.h>
#include<duckiepond/MotorCmd.h>
#include<math.h>
#include<Servo.h>

byte servoPin_R = 2;
byte servoPin_L = 3;
int rightSp = 1500;
int leftSp = 1500;
Servo servo_r;
Servo servo_l;
ros::NodeHandle nh;

void cbMotor(const duckiepond::MotorCmd& msg){
    rightSp = int(msg.right*400) + 1500;
    leftSp = int(msg.left*400) + 1500;
}

ros::Subscriber<duckiepond::MotorCmd> sub("motor_cmd",cbMotor);

void setup(){
    nh.initNode();
    nh.subscribe(sub);
    servo_r.attach(servoPin_R);
    servo_l.attach(servoPin_L);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);
    delay(7000);
}

void loop(){
    servo_r.writeMicroseconds(rightSp);
    servo_l.writeMicroseconds(leftSp);
    nh.spinOnce();
}
