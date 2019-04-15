// I²C library
#include <Wire.h>
// Timers library
#include <time.h>
// IMU sensor library
#include <TroykaIMU.h>
// SD card library
#include <SD.h>

// Connections:
// Encoder on LEFT wheel:
const byte encoder0pinA_LEFT = 3;// A pin = interupt pin 1
const byte encoder0pinB_LEFT = 4;// B pin
// Encoder on RIght wheel:
const byte encoder0pinA_RIGHT = 2; // A pin = interupt pin 0
const byte encoder0pinB_RIGHT = 5; // B pin

// Motor connections:
const int rightSpeed = 6; // PWM for right wheel rotation speed (pin 1 on L293D chip)
const int leftSpeed = 9; // PWM for left wheel rotation speed (pin 9 on L293D chip)
const int logic_0 = 7; // Wheels rotation direction. High - forward (pin 15 on L293D chip)
const int logic_1 = 10; // Wheels rotation direction. High - backward (pin 10 on L293D chip)

// SD card module:
const int CS_pin = 8;

// Accelerometer class object
Accelerometer accel;
// Gyroscope class object
Gyroscope gyro;

// LEFT encoder variables:
byte encoder0PinALast_LEFT; // the last value on A pin
boolean Direction_LEFT; // the rotation direction
int duration_LEFT = true; // the number of the pulses
void wheelSpeed_LEFT(); // encoder function

// RIGHT encoder variables:
byte encoder0PinALast_RIGHT; // the last value on A pin
int duration_RIGHT;// the number of the pulses
boolean Direction_RIGHT = true;// the rotation direction
void wheelSpeed_RIGHT(); // encoder function

// Motion control coefficients
float ka = 2.6; 
float kda = 0.22;
float kb = 1.9;
float kt = -0.002;

// Logger time
long start_time = 0;
// Static adjustments for engines imperfection
const int leftMotor_adj = -28; // static adjustment for left motor
const int rightMotor_adj = 0; // static adjustment for right motor
// main cycle operation time
const int cycle_time = 75; // ms
// maximum allowed rotation speed
const int max_w = 220;


void setup()
{
  // set pins mode:
  pinMode(rightSpeed, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(logic_0, OUTPUT);
  pinMode(logic_1, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(encoder0pinB_LEFT, INPUT);
  pinMode(encoder0pinB_RIGHT, INPUT);

  // start serial port:
  Serial.begin(57600); // Initialize the serial port

  // Initialize encoders
  attachInterrupt(0, wheelSpeed_RIGHT, CHANGE); // pin 2 on arduino
  attachInterrupt(1, wheelSpeed_LEFT, CHANGE); // pin 3 on arduino

  // Init gyroscope
  gyro.begin();
  // Set gyroscope sensivity mode
  gyro.setRange(RANGE_250DPS);
  // Init accelerometer
  accel.begin();
  // Set accelerometer sensivity mode
  accel.setRange(RANGE_2G);

  // create a start time mark
  start_time = millis();

  // Check if SD card is connected
  if (!SD.begin(CS_pin))
  {
    Serial.println("Card Failure");
    return;
  }
  Serial.println("Card Ready");
  // Check is Data.txt already exists
  if (SD.exists("Data.txt"))
    SD.remove("Data.txt");
  File logFile = SD.open("Data.txt", FILE_WRITE);
  // check if file was created successefuly
  if (logFile)
  {
    String header = "Tang, accX, Time, Velocity, Power_left, Power_right, Angle";
    logFile.println(header);
    logFile.close();
    Serial.println(header);
  }
  else
    Serial.println("Couldn't open log file");
}


void loop()
{
  float Tang = gyro.readDegPerSecX() * 3.14 / 180; // this is in radians
  float ACSEL = accel.readAY();
  float Kyrs = gyro.readDegPerSecZ();
  
  Serial.print("Duration left: ");
  Serial.print(duration_LEFT);
  Serial.print(", Duration right: ");
  Serial.println(duration_RIGHT);
  
  // convert velocity from odometer pulses to meters per second
  float frequency = 1000 / cycle_time;
  float Velocity = ( duration_LEFT + duration_RIGHT ) * 0.021 * 30 * frequency * 3.14 / 180 / 2;
  // Velocity control signal
  float control_Vy = ka * ACSEL + kda * Tang + kb * (Velocity);
  // Angular rate control signal
  float control_tetta = kt * (Kyrs - 0);
  
  Serial.print("ACSEL: ");
  Serial.print(ACSEL);  
  Serial.print(", Tang: ");
  Serial.print(Tang);
  Serial.print(", Velocity: ");
  Serial.println(Velocity);

  // Torque values for both engines, in N*m
  float Tau_l = (control_Vy - control_tetta) / 2;
  float Tau_r = (control_Vy + control_tetta) / 2;
  // PWM signal for both engines (in [0 255] range)
  float w_Tau_l =  10.5 * abs(Tau_l) + 110;
  float w_Tau_r =  10.5 * abs(Tau_r) + 110;

  // Set movement direction
  if (Tau_l > 0)
  {
    digitalWrite(logic_0, HIGH); // Move forward
    digitalWrite(logic_1, LOW);
  }
  else
  {
    digitalWrite(logic_0, LOW);
    digitalWrite(logic_1, HIGH); // Move backward
  }

  // limit w_Tau to max values and make sure they're decimal
  w_Tau_l = ceil(w_Tau_l);
  w_Tau_r = ceil(w_Tau_r);
  if (abs(w_Tau_l) > max_w)
    w_Tau_l = max_w;
  if (abs(w_Tau_r) > max_w)
    w_Tau_r = max_w;
  
  Serial.print("Tau_l: ");
  Serial.print(Tau_l);
  Serial.print(", Tau_r: ");
  Serial.print(Tau_r);
  Serial.print(", w_Tau_l: ");
  Serial.print(w_Tau_l);
  Serial.print(", w_Tau_r: ");
  Serial.println(w_Tau_r);

  // drop used encoder measurements
  duration_LEFT = 0;
  duration_RIGHT = 0;
  
  analogWrite(leftSpeed, abs(w_Tau_l) + leftMotor_adj); // just to make sure they are positive
  analogWrite(rightSpeed, abs(w_Tau_r) + rightMotor_adj);

  // get current time mark
  long cur_time = millis() - start_time; // timestamp
  // Write logs to the SD card
  String dataString = String(Tang) + ", " + String(ACSEL) + ", " + String(cur_time) + ", " + String(Velocity) + ", " + String(w_Tau_l) + ", " + String(w_Tau_r) + ", " + String(Kyrs);
  File logFile = SD.open("Data.txt", FILE_WRITE);
  if (logFile)
  {
    logFile.println(dataString);
    logFile.close();
    Serial.println(dataString);
  }

  // main cycle delay
  delay(cycle_time);
}


void wheelSpeed_LEFT()
{
  int Lstate_LEFT = digitalRead(encoder0pinA_LEFT);
  if ((encoder0PinALast_LEFT == LOW) && Lstate_LEFT == HIGH)
  {
    int val = digitalRead(encoder0pinB_LEFT);
    if (val == LOW && Direction_LEFT)
    {
      Direction_LEFT = false; //Reverse
    }
    else if (val == HIGH && !Direction_LEFT)
    {
      Direction_LEFT = true; //Forward
    }
  }
  else if ((encoder0PinALast_LEFT == HIGH) && Lstate_LEFT == LOW)
  {
    int val = digitalRead(encoder0pinB_LEFT);
    if (val == HIGH && Direction_LEFT)
    {
      Direction_LEFT = false; //Reverse
    }
    else if (val == LOW && !Direction_LEFT)
    {
      Direction_LEFT = true; //Forward
    }
  }
  encoder0PinALast_LEFT = Lstate_LEFT;
  if (!Direction_LEFT) duration_LEFT++;
  else duration_LEFT--;

}

void wheelSpeed_RIGHT()
{
  int Lstate_RIGHT = digitalRead(encoder0pinA_RIGHT);
  if ((encoder0PinALast_RIGHT == LOW) && Lstate_RIGHT == HIGH)
  {
    int val = digitalRead(encoder0pinB_RIGHT);
    if (val == LOW && !Direction_RIGHT)
    {
      Direction_RIGHT = true; //Reverse
    }
    else if (val == HIGH && Direction_RIGHT)
    {
      Direction_RIGHT = false; //Forward
    }
  }
  else if ((encoder0PinALast_RIGHT == HIGH) && Lstate_RIGHT == LOW)
  {
    int val = digitalRead(encoder0pinB_RIGHT);
    if (val == HIGH && !Direction_RIGHT)
    {
      Direction_RIGHT = true; //Reverse
    }
    else if (val == LOW && Direction_RIGHT)
    {
      Direction_RIGHT = false; //Forward
    }
  }
  encoder0PinALast_RIGHT = Lstate_RIGHT;
  if (!Direction_RIGHT) duration_RIGHT++;
  else duration_RIGHT--;

}
