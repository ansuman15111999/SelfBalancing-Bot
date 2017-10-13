
#include<Wire.h>

const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  

double compAngleX, compAngleY; 

#define degconvert 57.2957786 //there are like 57 degrees in a radian.
int motorOutput=0;

float Kp = 80;                   
float Ki = 0.0003;                       
float Kd = 0.5; 
      
float previousAngle;                // Keeps track of error over time
float targetAngle = -7;          // Can be adjusted according to centre of gravity 
float iTerm;
double thisTime = 0;
double lastTime = 0;

int rm1=9;
int rm2=10;


int lm1=5;
int lm2=6;


#define STD_LOOP_TIME 10
 
int timeGoneBy = STD_LOOP_TIME;
 
unsigned long loopStartTime = 0;


void setup() {
  // Set up MPU 6050:
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(230400);
  delay(100);

  //1)reading the mpu values
  read_mpu_registers();

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  //start a timer
  timer = micros();

delay(200);
}

void loop() {
  
float angle=getAngle();   // Go to the getAngle function it shows how it calculate the angle return the instant angle

motorOutput=PID(angle);   // it gives the output to the motor

// Serial.print("motorOutput :");
// Serial.println(motorOutput);



     if((angle<-70)||(angle>60))stopage(0);  // if the bot falls then it will stop automatically avoiding damage to the parts of bot  
                                             // stopage(0) : the integer inside function tells about the delay                                            
else if(angle<targetAngle)motorForward(10);  // function is motorForward(int myDelay) the INTEGER WITHIN bracket shows the delay after the execution of program

else if(angle>targetAngle)motorBackward(10);  // similar to above

else stopage(0);                            // it it is the target angle it will get automatically stopped


delay(0);
}




// ======================================================================
// ====                       MPU-6050 READINGS                      ====
// ======================================================================
void read_mpu_registers(){
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
}



// ======================================================================
// ====                           GET ANGLE                          ====
// ======================================================================
float getAngle(){
  
  read_mpu_registers();
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things:
                                                    //1) stops the timer 
                                                    //2)converts the timer's output to seconds from microseconds
                                                    //3)casts the value as a double saved to "dt"
                                                    
  timer = micros(); //start the timer again so that we can calculate the next dt.

  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

 
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;


  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; 
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 


  

//Serial.print(compAngleX);Serial.print("\t");
Serial.print(compAngleY);Serial.print("\n");

  return (compAngleY); 
}


// ======================================================================
// ====                       PID CONTROLLER                         ====
// ======================================================================

int PID(float instantAngle) {            

    float error = targetAngle - instantAngle;

    float pTerm = Kp * error;
    iTerm += Ki * error;
    float dTerm = Kd * (instantAngle - (1.2*previousAngle)) ;
   pTerm=abs(pTerm);
   pTerm=map(pTerm,0,2600,0,450);
//  Serial.print("pTerm : ");
//  Serial.println(pTerm);
//  Serial.print("iTerm : ");
//  Serial.println(iTerm);
//  Serial.print("dTerm : ");
//  Serial.println(dTerm);

    previousAngle = instantAngle;

    float PIDValue = pTerm + iTerm - dTerm-10;

    PIDValue=abs(PIDValue);         // converting the negative to positive as we need only the positive value 
    if (PIDValue > 255) PIDValue = 255;
 if(instantAngle>-7.5&&instantAngle<-6) PIDValue=0;   
 if( instantAngle<-60||instantAngle>60) PIDValue=0; 
  Serial.print("PIDValue: ");
  Serial.println(PIDValue);

 
    return int(PIDValue);
}



// ======================================================================
// ====                       MOTOR FUNCTION                         ====
// ======================================================================

void stopage(int myDelay){
  analogWrite(lm1,LOW);
  analogWrite(lm2,LOW);

  analogWrite(rm1,LOW);
  analogWrite(rm2,LOW);
  
  delay(myDelay);
}


void motorForward(int myDelay){

  analogWrite(lm1,motorOutput);
  analogWrite(lm2,LOW);

  analogWrite(rm1,motorOutput);
  analogWrite(rm2,LOW);

  delay(myDelay);
 }

void motorBackward(int myDelay){
  analogWrite(lm1,LOW);
  analogWrite(lm2,motorOutput);

  analogWrite(rm1,LOW);
  analogWrite(rm2,motorOutput);

  delay(myDelay);
 }

