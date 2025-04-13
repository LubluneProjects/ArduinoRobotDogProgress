
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//  PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int Servo1,Servo2;
int MinValue1, MidValue1;
int MinValue2, MidValue2;

int floorLvl = -13;  //floor cordinate for IK
int upLiftlevel= -8;

int del=300;  //delay

const float L1 = 7.4;  // femur
const float L2 = 7.4;   // tibia
 int bas=85;



//-------------------------------------------------------SERVO VALUES----------------------------------------------------------

#define SERVOMIN0 115   // LEFT FRONT femur servo 0 
#define SERVOMAX0 525   // LEFT FRONT femur servo  180

#define SERVOMIN1 215   // LEFT FRONT tibia servo  0 (leg straight)
#define SERVOMAX1 420   // LEFT FRONT tibia servo  90 

#define SERVOMIN2 112   // LEFT FRONT shoulder servo servomid for shoulder 
#define SERVOMAX2 312   // LEFT FRONT shoulder servo servomid for shoulder 



#define SERVOMIN3 131   // RIGHT FRONT femur servo   
#define SERVOMAX3 525   // RIGHT FRONT femur servo 

#define SERVOMIN4 200   // RIGHT FRONT tibia servo  Servomid for tibia  
#define SERVOMAX4 412   // RIGHT FRONT tibia servo  Servomid for tibia  

#define SERVOMIN5 317   // RIGHT FRONT shoulder servo servomid for shoulder 
#define SERVOMAX5 511   // RIGHT FRONT shoulder servo servomid for shoulder 



#define SERVOMIN6 115   // LEFT BACK femur servo 
#define SERVOMAX6 509   // LEFT BACK femur servo 

#define SERVOMIN7 250   // LEFT BACK tibia servo  Servomid for tibia 
#define SERVOMAX7 459   // LEFT BACK tibia servo  Servomid for tibia

#define SERVOMIN8 328   // LEFT BACK shoulder servo servomid for shoulder    
#define SERVOMAX8 525   // LEFT BACK shoulder servo servomid for shoulder 



#define SERVOMIN9 110   // RIGHT BACK femur servo   
#define SERVOMAX9 501   // RIGHT BACK femur servo

#define SERVOMIN10 176   // RIGHT BACK tibia servo  Servomid for tibia  
#define SERVOMAX10 393   // RIGHT BACK tibia servo  Servomid for tibia 

#define SERVOMIN11 117   // RIGHT BACK shoulder servo servomid for shoulder      
#define SERVOMAX11 317   // RIGHT BACK shoulder servo servomid for shoulder   

//-------------------------------------------------------SERVO VALUES----------------------------------------------------------



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pwm.begin();        // PCA9685
  pwm.setPWMFreq(50); //  50 Hz 
stand();

}



void loop() {

 Trot(5);

//  StandTrot();

}



//------------------------------------------------------------------------STAND-------------------------------------------------------
void stand() {
  for (int leg = 1; leg <= 4; leg++) {
    pwm.setPWM(Servo1, 0, IK(3, -11, leg, true));  // Femur to floor
    pwm.setPWM(Servo2, 0, IK(3, -11, leg, false)); // Tibia to floor
  }



  }


//------------------------------------------------------------------------IK-----------------------------------------------------
int IK(float x, float y, int legChosed,bool first){
int servo2Angle,servo1Angle;
y=-y;
      // ik
      float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
      //  c2 [-1, 1]
      c2 = constrain(c2, -1.0, 1.0);
      //s2=sqrt(1 - c2 * c2)
      float theta2 = atan2(sqrt(1 - c2 * c2), c2);  
      float k1 = L1 + L2 * cos(theta2);
      float k2 = L2 * sin(theta2);
      float theta1 = atan2(y, x) - atan2(k2, k1);

      int angle1 = theta1 * 180.0 / PI;
      int angle2 = theta2 * 180.0 / PI;

      if (legChosed == 1){
        
        Servo1=0;
        Servo2=1;
     servo1Angle = map(angle1, 180, 0,  SERVOMIN0, SERVOMAX0);
     servo2Angle = map(angle2, 90, 0, SERVOMIN1, SERVOMAX1);
      
      }else if(legChosed ==2){
    
        Servo1=3;
        Servo2=4;
 
     servo1Angle = map(angle1, 0, 180, SERVOMIN3 , SERVOMAX3);
     servo2Angle = map(angle2, 0, 90, SERVOMIN4 , SERVOMAX4);       
      }else if(legChosed ==3){
        
        Servo1=6;
        Servo2=7;
     servo1Angle = map(angle1, 180, 0, SERVOMIN6 , SERVOMAX6);
     servo2Angle = map(angle2, 90, 0, SERVOMIN7, SERVOMAX7);  
      }else if(legChosed ==4){

        Servo1=9;
        Servo2=10;
     servo1Angle = map(angle1, 0, 180, SERVOMIN9 , SERVOMAX9);
     servo2Angle = map(angle2, 0, 90, SERVOMIN10 , SERVOMAX10);
      }

if(first){
  return servo1Angle;
}else{
  return servo2Angle;
}

}
//----------------experimentel-------------------
int shoulderMap(int servoChosed,int angle){
  int ReturnedAngle;

if(servoChosed==1){
  ReturnedAngle=map(angle, 0, 90, SERVOMIN2 , SERVOMAX2);//LF shoulder
}else if(servoChosed==2){
  ReturnedAngle=map(angle, 90, 0, SERVOMIN5 , SERVOMAX5);//RF shoulder
}else if(servoChosed==3){
  ReturnedAngle=map(angle, 90, 0, SERVOMIN8 , SERVOMAX8);//LB shoulder
}else if(servoChosed==4){
  ReturnedAngle=map(angle, 0, 90, SERVOMIN11 , SERVOMAX11);//RB shoulder
}

return ReturnedAngle;
}
//--------------------one leg cycle-------------------
int Trot(int period){
float AX=-1;
float AY=-11;
float BX=3;
float BY=-11;
float CX=0;
float CY=-8;
float DX=2;
float DY=-11;

/* old 3point version
interpolation(BX,BY,CX,CY,AX,AY,BX,BY,20); // first leg B(x,y)--> C(x,y)  second leg A(x,y)-->B(x,y)
delay(period);
interpolation(CX,CY,AX,AY,BX,BY,BX,BY,20); //first leg C(x,y)-->A(x,y) second leg rest
delay(period);
//now after the first legs are touching the ground and are about to push themsolves backward the next legs shall start the cycle

interpolation(AX,AY,BX,BY,BX,BY,CX,CY,20); // first leg C(x,y)--> A(x,y)  second leg B(x,y)-->C(x,y)
delay(period);
interpolation(BX,BY,BX,BY,CX,CY,AX,AY,20); //first leg  rest second leg C(x,y)-->A(x,y)
delay(period);
*/

//new 4point version
interpolation(BX,BY,CX,CY,DX,DY,DX,DY,5); // first leg B(x,y)--> C(x,y)  second rest
delay(period);
interpolation(CX,CY,AX,AY,DX,DY,DX,DY,10); //first leg C(x,y)-->A(x,y) second leg rest
delay(period);
//now after the first legs are touching the ground and are about to push themsolves backward the next legs shall start the cycle


interpolation(AX,AY,DX,DY,DX,DY,BX,BY,20); //first leg A(x,y)--> D(x,y)  second D(x,y)--> B(x,y)
delay(period);
interpolation(DX,DY,DX,DY,BX,BY,CX,CY,5); //first leg rest  second B(x,y)--> C(x,y)
delay(period);
interpolation(DX,DY,DX,DY,CX,CY,AX,AY,10); //first leg rest  second C(x,y)--> A(x,y)
delay(period);
interpolation(DX,DY,BX,BY,AX,AY,DX,DY,20); //first leg D(x,y)--> B(x,y)  second A(x,y)--> D(x,y)
delay(period);




}


//-------------------standing trot-------------------

int StandTrot(){

float AX=3;
float AY=-6;
float BX=3;
float BY=-11;

interpolation(BX,BY,AX,AY,BX,BY,BX,BY,50); 
interpolation(AX,AY,BX,BY,BX,BY,BX,BY,50);

}



//----------------------------interpolation function------------------------
int interpolation(float startX1,float startY1, float endX1, float endY1, float startX2,float startY2, float endX2, float endY2 ,float steps){
  //this massive function takes A1(x,y)--->B1(x,y)   and for the second leg A2(x,y)---->B2(x,y)

  
  float s1start,s2start,s1end,s2end,s3start,s4start,s3end,s4end,s5start,s6start,s5end,s6end,s7start,s8start,s7end,s8end;
//these variables are servoControllerValues for start point and end point
//each leg has 2 servos (bypassing the shoulder) and each servo needs 2 variables, so in total each leg needs 4 variables (Servo1 start,Servo1 end,Servo2 start,Servo2 end)


  s1start=IK(startX1,startY1,1,true);
  s2start=IK(startX1,startY1,1,false);//left front leg
  s1end=IK(endX1,endY1,1,true);
  s2end=IK(endX1,endY1,1,false);

  s3start=IK(startX1,startY1,4,true);
  s4start=IK(startX1,startY1,4,false);//right back leg
  s3end=IK(endX1,endY1,4,true);
  s4end=IK(endX1,endY1,4,false);



  s5start=IK(startX2,startY2,2,true);
  s6start=IK(startX2,startY2,2,false);//right front leg
  s5end=IK(endX2,endY2,2,true);
  s6end=IK(endX2,endY2,2,false);

  s7start=IK(startX2,startY2,3,true);
  s8start=IK(startX2,startY2,3,false);//left back leg
  s7end=IK(endX2,endY2,3,true);
  s8end=IK(endX2,endY2,3,false);




float interpolatedPos1,interpolatedPos2,interpolatedPos3,interpolatedPos4,interpolatedPos5,interpolatedPos6,interpolatedPos7,interpolatedPos8;

    for(float i=0;i<steps;i++){
     interpolatedPos1 = s1start + (s1end-s1start)*i/steps;
    interpolatedPos2 = s2start + (s2end-s2start)*i/steps;
    
    interpolatedPos3 = s3start + (s3end-s3start)*i/steps;
    interpolatedPos4 = s4start + (s4end-s4start)*i/steps;

    interpolatedPos5 = s5start + (s5end-s5start)*i/steps;
    interpolatedPos6 = s6start + (s6end-s6start)*i/steps;
    
    interpolatedPos7 = s7start + (s7end-s7start)*i/steps;
    interpolatedPos8 = s8start + (s8end-s8start)*i/steps;


//    each leg gets it own coordinates



    pwm.setPWM(5, 0, shoulderMap(2, bas));  // Adjust LF
    pwm.setPWM(8, 0, shoulderMap(3, bas)); // Adjust RB
    pwm.setPWM(2, 0, shoulderMap(1, bas));  // Adjust LF
    pwm.setPWM(11, 0, shoulderMap(4, bas)); // Adjust RB



    
    pwm.setPWM(0, 0, interpolatedPos1);  
    pwm.setPWM(1,0, interpolatedPos2);
    pwm.setPWM(9, 0, interpolatedPos3);  
    pwm.setPWM(10,0, interpolatedPos4);

    pwm.setPWM(3, 0, interpolatedPos5);  
    pwm.setPWM(4,0, interpolatedPos6);
    pwm.setPWM(6, 0, interpolatedPos7);  
    pwm.setPWM(7,0, interpolatedPos8);
  }


}
