#include <QTRSensors.h>

int ENA = 8; //ENA connected to digital pin 9
int ENB = 9; //ENB connected to digital pin 3
int MOTOR_A1 = 10; // MOTOR_A1 connected to digital pin 7
int MOTOR_A2 = 11; // MOTOR_A2 connected to digital pin 6
int MOTOR_B1 = 12; // MOTOR_B1 connected to digital pin 5
int MOTOR_B2 = 13; // MOTOR_B2 connected to digital pin 4

int lastError = 0;

float Kp = 0.21;
float Ki = 0;
float Kd = 0.0;

int baseSpeedValue = 90;

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

void setup() {

  pinMode(ENA, OUTPUT); // initialize ENA pin as an output
  pinMode(ENB, OUTPUT); // initialize ENB pin as an output
  pinMode(MOTOR_A1, OUTPUT); // initialize MOTOR_A1 pin as an output
  pinMode(MOTOR_A2, OUTPUT); // initialize MOTOR_A2 pin as an output
  pinMode(MOTOR_B1, OUTPUT); // initialize MOTOR_B1 pin as an output
  pinMode(MOTOR_B2, OUTPUT); // initialize MOTOR_B2 pin as an output

  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  for (uint16_t i = 0; i < 200; i++)  //10 seconds
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(2500);
}

void loop() {
  // put your main code here, to run repeatedly:
     PID_Control();    
}


void PID_Control()
{
  uint16_t positionLine = qtr.readLineWhite(sensorValues);

  int error = 1500 - positionLine;
  
  int P = error;
  int I = error + I;
  int D = lastError - error;  
  lastError = error;

  int motorSpeedNew = P * Kp + I * Ki + D * Kd;
  
   int motorSpeedA = baseSpeedValue + motorSpeedNew;
   int motorSpeedB = baseSpeedValue - motorSpeedNew;

   if(motorSpeedA > 185 )
   motorSpeedA = 185;

   if(motorSpeedB > 185 )
   motorSpeedB = 185;

   if(motorSpeedA < 0 )
   motorSpeedA = 0;

   if(motorSpeedB < 0 )
   motorSpeedB = 0;

   movement(motorSpeedA, motorSpeedB);
}


void movement(int speedA, int speedB)
{
if(speedA < 0)
{
  speedA = 0 - speedA;
  analogWrite(ENA, speedA);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
}

else
{
  analogWrite(ENA, speedA);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
}

if(speedB < 0)
{
  speedB = 0 - speedB;
  analogWrite(ENB, speedB); 
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

else
{
  analogWrite(ENB, speedB); 
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

}