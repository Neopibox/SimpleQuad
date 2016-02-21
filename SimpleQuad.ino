#include <Console.h>
#include <SerialCommand.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#define RUN

// Axis name
const uint8_t X = 0;
const uint8_t Y = 1;
const uint8_t Z = 2;

// Throttle limits
const uint16_t MIN_THROTTLE = 1150*2;
const uint16_t MAX_THROTTLE = 1792*2;

// PID's
float sumerrX = 0;
float sumerrY = 0;

// Parameters

// kP = 0.60Ku
// kI = 2Kp/Tu
// kD = 0.15KpTu

//float kP = 0.15, kI = 0.05, kD = 0.15; // kP JAMAIS A 1 BORDEL !
float kP = 0.62, kI = 0.03, kD = 0.08; // kP JAMAIS A 1 BORDEL !

// Motors
#define motor1 OCR1C  // PIN 11
#define motor2 OCR1B  // PIN 10
#define motor3 OCR1A  // PIN 9
#define motor4 OCR3A  // PIN 5

// Time
uint16_t previousMicros = 0;
uint16_t currentMicros = 0;

// Quadcopter orientation
float angle[3] = {0};

// Serial
SerialCommand sCmd;

uint8_t isArmed = 0;

// Setpoint values
float s_throttle = 0;
float s_pitch = 0;
float s_roll = 0;
float s_yaw = 0;

double dT;
uint16_t frequency;

L3G gyro;
LSM303 compass;

//Processing sensors values
float gyro_angle[3] = {0};
float accl_angle[3] = {0};

char fails = 0;

/*long timer, printTimer;
float G_Dt;
int loopCount;

float q0;
float q1;
float q2;
float q3;
float beta;

float magnitude;

//float pitch,roll,yaw;

float gyroSumX,gyroSumY,gyroSumZ;
float offSetX,offSetY,offSetZ;

float floatMagX,floatMagY,floatMagZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;*/

/* --- SERIAL HANDLERS --- */

void setP()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    float P = atof(arg);
    kP = P;
    
    Console.print(F("kP set to "));
    Console.println(kP);
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
    
    Console.print(F("kI set to "));
    Console.println(kI);
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
    
    Console.print(F("kD set to "));
    Console.println(kD);
  }
}

void armMotors()
{
  isArmed = 1;
  Console.println(F("Quadcopter armed ! Ready to fly !"));
}

void disarmMotors()
{
  isArmed = 0;
  Console.println(F("Quadcopter disarmed ! Shutting down motors ..."));
}

void show()
{
    Console.print(F("kP set to "));
    Console.println(kP);
    Console.print(F("kI set to "));
    Console.println(kI); 
    Console.print(F("kD set to "));
    Console.println(kD);
}

void setThrottle()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    s_throttle = atoi(arg);
    
    Console.print(F("Throttle set to "));
    Console.println(s_throttle);
  }
}

void setPitch()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    s_pitch = atof(arg);
    
    Console.print(F("Pitch set to "));
    Console.println(s_pitch);
  }
}

void setRoll()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    s_roll = atof(arg);
        
    Console.print(F("Roll set to "));
    Console.println(s_roll);
  }
}

void setYaw()
{
  char *arg;
  arg = sCmd.next();

  if(arg != NULL)
  {
    s_yaw = atof(arg);
    
    Console.print(F("Yaw set to "));
    Console.println(s_yaw);
  }
}

void rsti()
{
  sumerrX = 0;
  sumerrY = 0;
}

void unknownCommand(const char *command)
{
  Console.println(F("Unkown command"));
}

/* --- PID --- */

float PIDX(float err, float dT)
{
  register float P = 0, I = 0, D = 0;
  static float lasterrX = 0;
  
  sumerrX += (err * dT);
  
  P = err * kP;
  I = sumerrX * kI;
  D = ((err - lasterrX)/dT) * kD;
  
  lasterrX = err;
  return P + I + D;
}

float PIDY(float err, float dT)
{
  register float P = 0, I = 0, D = 0;
  static float lasterrY = 0;
  
  sumerrY += (err * dT);
  
  P = err * kP;
  I = sumerrY * kI;
  D = ((err - lasterrY)/dT) * kD;
  
  lasterrY = err;
  return P + I + D;
}

/* --- MOTORS --- */

void dispatchMotors(float thr, float a_pitch, float a_roll, float a_yaw)
{
  // Convert actions into speed
  float thr1 = (thr - a_pitch + a_roll - a_yaw);
  float thr2 = (thr - a_pitch - a_roll + a_yaw);
  float thr3 = (thr + a_pitch + a_roll + a_yaw);
  float thr4 = (thr + a_pitch - a_roll - a_yaw);

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

  // Convert throttle into time
  thr1 = MIN_THROTTLE + thr1 * ((MAX_THROTTLE - MIN_THROTTLE) / 100);
  thr2 = MIN_THROTTLE + thr2 * ((MAX_THROTTLE - MIN_THROTTLE) / 100);
  thr3 = MIN_THROTTLE + thr3 * ((MAX_THROTTLE - MIN_THROTTLE) / 100);
  thr4 = MIN_THROTTLE + thr4 * ((MAX_THROTTLE - MIN_THROTTLE) / 100);
  
//  // 0 is a "stop motors" value
//  if(thr1 == 0)
//    thr1 = 0;
//  else if(thr2 == 0)
//    thr2 = 0;
//  else if(thr3 == 0)
//    thr3 = 0;
//  else if(thr4 == 0)
//    thr4 = 0;
  
  writeMotors((uint16_t) thr1, (uint16_t) thr2, (uint16_t) thr3, (uint16_t) thr4);
}

void writeMotors(uint16_t time1, uint16_t time2, uint16_t time3, uint16_t time4)
{
  motor1 = time1;
  motor2 = time2;
  motor3 = time3;
  motor4 = time4;
}

/* --- INIT --- */

void initMotors()
{
  // Timer 1
  DDRB |= (1 << 7) | (1 << 6) | (1 << 5); // PIN 9, 10, 11 OUTPUT
  ICR1 = 0x1387;  // 400Hz

  TCCR1A = 0b10101010;
  TCCR1B = 0b00011010;

  motor1 = 0;
  motor2 = 0;
  motor3 = 0;

  // Timer 3
  DDRC |= (1 << PC6); // PIN 5 OUTPUT
  ICR3 = 0x1387;  // 400Hz

  TCCR3A = 0b10101010;
  TCCR3B = 0b00011010;

  motor4 = 0;
}

void setup()
{
  // ----- Initialization -----

  // -- Serial Init --
  Serial.begin(115200);
  Bridge.begin();
  Console.begin();

  while (!Console)
  {
    
  }

  sCmd.addCommand("ARM", armMotors);        // Arm Motors
  sCmd.addCommand("DISARM", disarmMotors);  // Disarm Motors
  sCmd.addCommand("THR", setThrottle);      // Throttle
  sCmd.addCommand("PITCH", setPitch);       // Pitch
  sCmd.addCommand("ROLL", setRoll);         // Roll
  sCmd.addCommand("YAW", setYaw);           // Yaw
  sCmd.addCommand("KP", setP);              // kP
  sCmd.addCommand("KI", setI);              // kI
  sCmd.addCommand("KD", setD);              // kD
  sCmd.addCommand("SHOW", show);            // Show information
  sCmd.addCommand("RSTI", rsti);            // Reset I on PID's controllers
  sCmd.setDefaultHandler(unknownCommand);   // Others cases

  // -- Quadcopter Init --
  
  // AHRS Init
  Wire.begin();
  TWBR = ((F_CPU / 400000) - 16) / 2; //set the I2C speed to 400KHz
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  
  gyro.init();
  compass.init();
  
  gyro.enableDefault();
  compass.enableDefault();

  gyro.setTimeout(10);
  compass.setTimeout(10);

  compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
  //gyro.writeReg(L3G::CTRL4, 0x30); // 2000 dps full scale: FS = 11

  // Motors Init
  initMotors();
  dispatchMotors(0, 0, 0, 0); // Init ESCs
  delay(1000);

  previousMicros = millis();
}

void loop()
{
  sCmd.readSerial();  // Check for commands
  
  currentMicros = millis();
  dT = (currentMicros - previousMicros) / 1000.0;
  frequency = 1 / dT;
  previousMicros = currentMicros;
  
  // Get raw data
  gyro.read();
  compass.read();

  if(!gyro.timeoutOccurred() && !compass.timeoutOccurred()) // If one read timeout, freeze IMU
  {
    fails = 0;
    
    // Scale values
    float gyro_scaled[3] = {0};
    float accl_scaled[3] = {0};
    
    gyro_scaled[X] = gyro.g.x * 0.00875;
    gyro_scaled[Y] = gyro.g.y * 0.00875;
    gyro_scaled[Z] = gyro.g.z * 0.00875;
    
    accl_scaled[X] = compass.a.x * 0.000244;
    accl_scaled[Y] = compass.a.y * 0.000244;
    accl_scaled[Z] = compass.a.z * 0.000244;
  
    //Processing sensors values
    gyro_angle[X] = angle[Y] + (-gyro_scaled[X]) * dT;
    gyro_angle[Y] = angle[X] + (-gyro_scaled[Y]) * dT;
    gyro_angle[Z] = gyro_angle[Z] + gyro_scaled[Z] * dT;
  
    float square_z = accl_scaled[Z]*accl_scaled[Z];
    accl_angle[X] = -(atan2(accl_scaled[Y], sqrt(accl_scaled[X]*accl_scaled[X] + square_z))) * 57.2957795;
    accl_angle[Y] = atan2(accl_scaled[X], sqrt(accl_scaled[Y]*accl_scaled[Y] + square_z)) * 57.2957795;
  
    float alpha = 1/(1+dT);
    
    // Complementary filter
    angle[X] = alpha * gyro_angle[Y] + (1 - alpha) * accl_angle[Y];
    angle[Y] = alpha * gyro_angle[X] + (1 - alpha) * accl_angle[X];
  
    #ifdef DEBUG
    Console.print(F("G | X: "));
    Console.print(gyro_angle[X]);
    Console.print(F(" Y: "));
    Console.print(gyro_angle[Y]);
    
    Console.print(F(" || A | X: "));
    Console.print(accl_angle[X]);
    Console.print(F(" Y: "));
    Console.print(accl_angle[Y]);
    
    Console.print(F(" || F | X: "));
    Console.print(angle[X]);
    Console.print(F(" Y: "));
    Console.println(angle[Y]);
    #endif
  }
  else
  {
    fails++;
  }
    
  // Process stabilisation
  float Xaction = PIDX(s_pitch - angle[X], dT);
  float Yaction = PIDY(s_roll - angle[Y], dT);

  if(isArmed && fails < 10 )
  {
    // Send action to motors
    dispatchMotors(s_throttle, Xaction, Yaction, s_yaw);
  }
  else
  {
    writeMotors(MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE); // Shut down motors to prevent damage
  }

  #ifdef SHOW
  Console.print(F("F | X: "));
  Console.print(angle[X]);
  Console.print(F(" Y: "));
  Console.print(angle[Y]);
  
  Console.print(F(" || ACTION | X: "));
  Console.print(Xaction);
  Console.print(F(" Y: "));
  Console.println(Yaction);
  #endif
}
