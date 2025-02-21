


#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//  PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int Servo1,Servo2;
int MinValue1, MidValue1;
int MinValue2, MidValue2;

int floorLvl = -12;  //floor cordinate for IK
int del=500;  //delay


const float L1 = 7.4;  // femur
const float L2 = 7.4;   // tibia



//only femur and tibia servos are used in this code meaning that pca9685 pins 0,1,3,4,6,7,9,10 are used
//0-left front upper servo
//1-left front lower servo
//3-right front upper servo
//4-right front lower servo
//6-left back upper servo
//7-left back lower servo
//9-right back upper servo
//10-right back lower servo


void setup() {

  Serial.begin(9600);
  pwm.begin();        // PCA9685
  pwm.setPWMFreq(50); //  50 Hz 

int SerI = IK(0,-6,1,true);
int SerII = IK(0,-6,1,false);
int SerIII = IK(-2,floorLvl,2,true);
int SerIV = IK(-2,floorLvl,2,false);
 int SerV = IK(-4,floorLvl,3,true);
 int SerVI = IK(-4,floorLvl,3,false);
 int SerVII = IK(8,floorLvl+1,4,true);
 int SerVIII = IK(8,floorLvl+1,4,false);



//set the servos to the first position

pwm.setPWM(0, 0, SerI);
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV);
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 


       
}

void loop() {
  //left front leg up
int SerI = IK(0,-6,1,true);
int SerII = IK(0,-6,1,false);
int SerIII = IK(-2,floorLvl,2,true);
int SerIV = IK(-2,floorLvl,2,false);
 int SerV = IK(0,floorLvl,3,true);
 int SerVI = IK(0,floorLvl,3,false);
 int SerVII = IK(8,floorLvl+1,4,true);
 int SerVIII = IK(8,floorLvl+1,4,false);

pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);

//left front leg forward

 SerI = IK(-5,floorLvl-1,1,true);
 SerII = IK(-5,floorLvl-1,1,false);
 SerIII = IK(-2,floorLvl,2,true);
 SerIV = IK(-2,floorLvl,2,false);
 SerV = IK(0,floorLvl,3,true);
 SerVI = IK(0,floorLvl,3,false);
 SerVII = IK(8,floorLvl+1,4,true);
 SerVIII = IK(8,floorLvl+1,4,false);

pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);

//move body forward
 SerI = IK(-2,floorLvl,1,true);
 SerII = IK(-2,floorLvl,1,false);
 SerIII = IK(1,floorLvl,2,true);
 SerIV = IK(1,floorLvl,2,false);
 SerV = IK(3,floorLvl,3,true);
 SerVI = IK(3,floorLvl,3,false);
 SerVII = IK(11,floorLvl,4,true);
 SerVIII = IK(11,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//right back leg up
 SerI = IK(-2,floorLvl+1,1,true);
 SerII = IK(-2,floorLvl+1,1,false);
 SerIII = IK(1,floorLvl,2,true);
 SerIV = IK(1,floorLvl,2,false);
 SerV = IK(3,floorLvl,3,true);
 SerVI = IK(3,floorLvl,3,false);
 SerVII = IK(0,-6,4,true);
 SerVIII = IK(0,-6,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//right back leg forward
 SerI = IK(-2,floorLvl+1,1,true);
 SerII = IK(-2,floorLvl+1,1,false);
 SerIII = IK(1,floorLvl,2,true);
 SerIV = IK(1,floorLvl,2,false);
 SerV = IK(3,floorLvl,3,true);
 SerVI = IK(3,floorLvl,3,false);
 SerVII = IK(-5,floorLvl-1,4,true);
 SerVIII = IK(-5,floorLvl-1,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//move body forward
 SerI = IK(1,floorLvl,1,true);
 SerII = IK(1,floorLvl,1,false);
 SerIII = IK(4,floorLvl,2,true);
 SerIV = IK(4,floorLvl,2,false);
 SerV = IK(6,floorLvl,3,true);
 SerVI = IK(6,floorLvl,3,false);
 SerVII = IK(-2,floorLvl,4,true);
 SerVIII = IK(-2,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//right front leg up
 SerI = IK(1,floorLvl,1,true);
 SerII = IK(1,floorLvl,1,false);
 SerIII = IK(0,-6,2,true);
 SerIV = IK(0,-6,2,false);
 SerV = IK(6,floorLvl+1,3,true);
 SerVI = IK(6,floorLvl+1,3,false);
 SerVII = IK(-2,floorLvl,4,true);
 SerVIII = IK(-2,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//right front leg forward
 SerI = IK(1,floorLvl,1,true);
 SerII = IK(1,floorLvl,1,false);
 SerIII = IK(-5,floorLvl-1,2,true);
 SerIV = IK(-5,floorLvl-1,2,false);
 SerV = IK(6,floorLvl+1,3,true);
 SerVI = IK(6,floorLvl+1,3,false);
 SerVII = IK(-2,floorLvl,4,true);
 SerVIII = IK(-2,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
delay(100);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//move body forward
 SerI = IK(4,floorLvl,1,true);
 SerII = IK(4,floorLvl,1,false);
 SerIII = IK(-2,floorLvl,2,true);
 SerIV = IK(-2,floorLvl,2,false);
 SerV = IK(9,floorLvl,3,true);
 SerVI = IK(9,floorLvl,3,false);
 SerVII = IK(1,floorLvl,4,true);
 SerVIII = IK(1,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//left back leg up
 SerI = IK(4,floorLvl,1,true);
 SerII = IK(4,floorLvl,1,false);
 SerIII = IK(-2,floorLvl+1,2,true);
 SerIV = IK(-2,floorLvl+1,2,false);
 SerV = IK(0,-6,3,true);
 SerVI = IK(0,-6,3,false);
 SerVII = IK(4,floorLvl,4,true);
 SerVIII = IK(4,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//left back leg forward
 SerI = IK(4,floorLvl,1,true);
 SerII = IK(4,floorLvl,1,false);
 SerIII = IK(-2,floorLvl+1,2,true);
 SerIV = IK(-2,floorLvl+1,2,false);
 SerV = IK(-5,floorLvl-1,3,true);
 SerVI = IK(-5,floorLvl-1,3,false);
 SerVII = IK(4,floorLvl,4,true);
 SerVIII = IK(4,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);
//move body forward
 SerI = IK(7,floorLvl,1,true);
 SerII = IK(7,floorLvl,1,false);
 SerIII = IK(0,floorLvl,2,true);
 SerIV = IK(0,floorLvl,2,false);
 SerV = IK(-2,floorLvl,3,true);
 SerVI = IK(-2,floorLvl,3,false);
 SerVII = IK(8,floorLvl,4,true);
 SerVIII = IK(8,floorLvl,4,false);
pwm.setPWM(0, 0, SerI); 
pwm.setPWM(1, 0, SerII); 
pwm.setPWM(3, 0, SerIII);
pwm.setPWM(4, 0, SerIV);

pwm.setPWM(6, 0, SerV); 
pwm.setPWM(7, 0, SerVI); 
pwm.setPWM(9, 0, SerVII); 
pwm.setPWM(10, 0, SerVIII); 
delay(del);

}

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
        MinValue2=500;
        MidValue2=290;  
        MinValue1=490;
        MidValue1=275;  
     servo1Angle = map(angle1, 0, 90, MinValue1 , MidValue1);
     servo2Angle = map(angle2, 0, 90, MinValue2, MidValue2);
      
      }else if(legChosed ==2){
    
        Servo1=3;
        Servo2=4;
        MinValue1=90;
        MidValue1=315;   
        MinValue2=95;
        MidValue2=290;   
     servo1Angle = map(angle1, 0, 90, MinValue1 , MidValue1);
     servo2Angle = map(angle2, 0, 90, MinValue2 , MidValue2);       
      }else if(legChosed ==3){
        
        Servo1=6;
        Servo2=7;//220 430
        MinValue2=490;
        MidValue2=285;   
        MinValue1=500;
        MidValue1=300;  
     servo1Angle = map(angle1, 0, 90, MinValue1 , MidValue1);
     servo2Angle = map(angle2, 0, 90, MinValue2, MidValue2);  
      }else if(legChosed ==4){
       
        Servo1=9;
        Servo2=10;
        MinValue2=105;
        MidValue2=310;   
        MinValue1=100;//////
        MidValue1=310; 
     servo1Angle = map(angle1, 0, 90, MinValue1 , MidValue1);
     servo2Angle = map(angle2, 0, 90, MinValue2 , MidValue2);
      }

if(first){
  return servo1Angle;
}else{
  return servo2Angle;
}



}
