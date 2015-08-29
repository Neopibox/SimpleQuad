#include <SPI.h>

#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <WiFiServer.h>

#include <SerialCommand.h>
#include <MPU9150.h>

#include <Servo.h>
#include <Wire.h>
#include <math.h>

int i = 0;

// Axis name
#define X 0
#define Y 1
#define Z 2

// Throttle limits
#define MIN_THROTTLE 1105
#define MAX_THROTTLE 1455

// Parameters

// kP = 0.60Ku
// kI = 2Kp/Tu
// kD = 0.15KpTu

float kP = 0.15, kI = 0.05, kD = 0.15; // kP JAMAIS A 1 BORDEL !
//float kP = 0.25, kI = 0.05, kD = 0.10; // kP JAMAIS A 1 BORDEL !

// Motors
Servo ESC1, ESC2, ESC3, ESC4;

// Time
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// Quadcopter orientation
double gyro_angle[3] = {0};
double accl_angle[3] = {0};
double angle[3] = {0};

MPU9150 mpu;

// Serial
SerialCommand sCmd;

char emergencyStop = 0;

/* --- SERIAL HANDLERS --- */

void setP()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    float P = atof(arg);
    kP = P;
    
    Serial.print("kP set to ");
    Serial.println(kP);
  }
}

void setI()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    float I = atof(arg);
    kI = I;
    
    Serial.print("kI set to ");
    Serial.println(kI);
  }
}

void setD()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    float D = atof(arg);
    kD = D;
    
    Serial.print("kD set to ");
    Serial.println(kD);
  }
}

void powerOff()
{
  emergencyStop = 1;
  Serial.println("Emergency stop has been actived ! Shutting down ...");
}

void showPID()
{
    Serial.print("kP set to ");
    Serial.println(kP);
    Serial.print("kI set to ");
    Serial.println(kI); 
    Serial.print("kD set to ");
    Serial.println(kD);
    
    delay(100);
}


void unknownCommand(const char *command)
{
  Serial.println("Unkown command");
}

/* --- UTILS --- */

float rangeThrottle(float value)
{
  return MIN_THROTTLE + value * ((MAX_THROTTLE - MIN_THROTTLE) / 100);
}

/* --- PID --- */

float PID(float err, float dT)
{
  float P = 0, I = 0, D = 0;
  static float sumerr = 0;
  static float lasterr = 0;
  
  sumerr += (err * dT);
  
  P = err * kP;
  I = sumerr * kI;
  D = ((err - lasterr)/dT) * kD;
  
  lasterr = err;
  return P + I + D;
}

/* --- MOTORS --- */

void dispatchMotors(float thr, float pitch, float roll, float yaw)
{
  // Convert actions into speed
  float thr1 = (thr - pitch + roll - yaw);
  float thr2 = (thr - pitch - roll + yaw);
  float thr3 = (thr + pitch + roll + yaw);
  float thr4 = (thr + pitch - roll - yaw);

  // Constraint values
  if(thr1 <= 0)
    thr1 = 0;
  else if(thr1 >= 100)
    thr1 = 100;
  
  if(thr2 <= 0)
    thr2 = 0;
  else if(thr2 >= 100)
    thr2 = 100;
    
  if(thr3 <= 0)
    thr3 = 0;
  else if(thr3 >= 100)
    thr3 = 100;
    
  if(thr4 <= 0)
    thr4 = 0;
  else if(thr4 >= 100)
    thr4 = 100;
  
  refreshMotors(thr1, thr2, thr3, thr4);
}

void refreshMotors(float thr1, float thr2, float thr3, float thr4)
{
  // Convert throttle into time
  float value1 = rangeThrottle(thr1), value2 = rangeThrottle(thr2), value3 = rangeThrottle(thr3), value4 = rangeThrottle(thr4);

  // 0 is a "stop motors" value
  if(thr1 == 0)
    value1 = 0;
  else if(thr2 == 0)
    value2 = 0;
  else if(thr3 == 0)
    value3 = 0;
  else if(thr4 == 0)
    value4 = 0;

  // Send motors' values
  writeMotors((int) value1, (int) value2, (int) value3, (int) value4);
}

void writeMotors(int time1, int time2, int time3, int time4)
{
  ESC1.writeMicroseconds(time1);
  ESC2.writeMicroseconds(time2);
  ESC3.writeMicroseconds(time3);
  ESC4.writeMicroseconds(time4);
}

/* --- INIT --- */

void MPUInit()
{
  mpu.Init();
  mpu.setAccelScale(4);
  mpu.setGyroScale(1000);

  delay(2000);
}

void MotorsInit()
{
  ESC1.attach(9);
  ESC2.attach(6);
  ESC3.attach(5);
  ESC4.attach(3);
  delay(15);

  writeMotors(0, 0, 0, 0); // Init ESCs

  delay(3000);
}

void setup()
{
  // Initialization

  // Serial Init
  Serial.begin(115200);
  sCmd.addCommand("P", setP);             // kP
  sCmd.addCommand("I", setI);             // kI
  sCmd.addCommand("D", setD);             // kD
  sCmd.addCommand("S", showPID);          // Show PID's coeffs
  sCmd.addCommand("STOP", powerOff);      // Emergency stop
  sCmd.setDefaultHandler(unknownCommand); // Others cases

  // Quadcopter Init
  MPUInit();
  MotorsInit();
  
  previousMillis = millis();
}

void loop()
{
  sCmd.readSerial();  // Check for commands
  
  currentMillis = millis();
  double dT = (currentMillis - previousMillis) / 1000.0;
  double frequency = 1 / dT;
  previousMillis = currentMillis;
  
  // Get raw data
  mpu.getScaled();
  
  //Processing sensors values
  gyro_angle[X] = angle[X] + mpu.gyroVector[X] * dT;
  gyro_angle[Y] = angle[Y] + -(mpu.gyroVector[Y]) * dT;
  gyro_angle[Z] = gyro_angle[Z] + mpu.gyroVector[Z] * dT;
  
  accl_angle[X] = atan2(-mpu.accelVector[Y], sqrt(square(-mpu.accelVector[X]) + square(mpu.accelVector[Z]))) * 57.2957795;
  accl_angle[Y] = atan2(-mpu.accelVector[X], sqrt(square(-mpu.accelVector[Y]) + square(mpu.accelVector[Z]))) * 57.2957795;
  
  // Complementary filter
  angle[X] = 0.96 * gyro_angle[X] + 0.04 * accl_angle[X];
  angle[Y] = 0.96 * gyro_angle[Y] + 0.04 * accl_angle[Y];
  
  // Process stabilisation
  float Xaction = PID(0 + angle[X], dT);
  float Yaction = PID(0 + angle[Y], dT);

  if(!emergencyStop)
  {
    // Send action to motors
    dispatchMotors(40, (int) Xaction, (int) Yaction, 0);
  }
  else
  {
    writeMotors(0, 0, 0, 0); // Shut down motors to prevent damage
  }

  if((i % 2) == 0 )
  {
    
  Serial.print(frequency);
  Serial.print("Hz,");
  Serial.print(angle[0]);
  Serial.print(",");
  Serial.print(angle[1]);
  Serial.print(",");
  Serial.print(Xaction);
  Serial.print(",");
  Serial.print(Yaction);
/*  Serial.print(",");
  Serial.print();  */
 Serial.println();
  }

  i++;
}
