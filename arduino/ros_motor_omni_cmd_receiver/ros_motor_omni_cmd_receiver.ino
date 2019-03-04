#include<ros.h>
#include<duckiepond/Motor4Cmd.h>
#include<math.h>

#define LFS 2
#define LFV 3

#define LRS 4
#define LRV 5

#define RFS 8
#define RFV 9

#define RRS 10
#define RRV 11

ros::NodeHandle nh;

void cbMotor(duckiepond::Motor4Cmd &msg){
  
    int lfv = int(abs(msg.lf*255));
    int lrv = int(abs(msg.lr*255));
    int rfv = int(abs(msg.rf*255));
    int rrv = int(abs(msg.rr*255));
    
    if(msg.lf < 0) digitalWrite(LFS,HIGH);
    else digitalWrite(LFS,LOW);
    if(msg.lr < 0) digitalWrite(LRS,HIGH);
    else digitalWrite(LRS,LOW);
    if(msg.rf > 0) digitalWrite(RFS,HIGH);
    else digitalWrite(RFS,LOW);
    if(msg.rr > 0) digitalWrite(RRS,HIGH);
    else digitalWrite(RRS,LOW);

    analogWrite(LFV,lfv);
    analogWrite(LRV,lrv);
    analogWrite(RFV,rfv);
    analogWrite(RRV,rrv);
}

ros::Subscriber<duckiepond::Motor4Cmd> sub("motor_cmd",cbMotor);

void setup(){
    nh.initNode();
    nh.subscribe(sub);

    pinMode(LFV,OUTPUT);
    pinMode(LFS,OUTPUT);
    pinMode(LRV,OUTPUT);
    pinMode(LRS,OUTPUT);
    
    pinMode(RFV,OUTPUT);
    pinMode(RFS,OUTPUT);
    pinMode(RRV,OUTPUT);
    pinMode(RRS,OUTPUT);

}



void loop(){
    nh.spinOnce();
}