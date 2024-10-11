#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint16_t threshold = 1250;
uint8_t binaryValues[SensorCount];   // Array to store '0' or '1' based on the threshold

float Kp = 0.04; // Set up the constants value 0.0575
float Ki = 0;
float Kd = 0.06;//0.1
int P;
int I;
int D;
int lastError = 0;
boolean onoff = true;

// Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7.4 V 
const uint8_t maxspeeda = 140;//to make fast put 230
const uint8_t maxspeedb = 240;//to make fast put 230
const uint8_t basespeeda = 60;//to make fast put 230
const uint8_t basespeedb = 120;//to make fast put 230

// Set up the drive motor carrier pins for TB6612FNG
int PWMA = 6;
int AIN1 = 9;
int AIN2 = 8;
int PWMB = 3;
int BIN1 = 5;
int BIN2 = 4;
int startLED = 10;

// Set up the buttons pins
int buttoncalibrate = 17; // Pin A3
int buttonstart = 2;

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  // Set up the sensor array pins
  qtr.setSensorPins((const uint8_t[]){11, 12, 14, 15, 16, 17, 18, 19}, SensorCount);
  qtr.setEmitterPin(7); // LEDON PIN

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(startLED, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false) { // The loop won't start until the robot is calibrated
    if(digitalRead(buttoncalibrate) == HIGH) {
      calibration(); // Calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  forward_brake(0, 0);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(startLED,5);
  delay(3000);
  analogWrite(startLED,0);
}

void loop() {
  unsigned int sensors[8];
  uint16_t position = qtr.readLineWhite(sensorValues);
  int error = position - 3500;
  /*Serial.print(position);
  Serial.print("  ");
  Serial.print(error);
  Serial.println();*/

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int motorspeeda = basespeeda - motorSpeed;
  int motorspeedb = basespeedb + motorSpeed;
  /*Serial.print(" a:");
  Serial.print(motorspeeda);
  Serial.print(" b:");
  Serial.print(motorspeedb);
  Serial.println();*/

  if (motorspeeda > maxspeeda) motorspeeda = maxspeeda; // prevent the motor from going beyond max speed
  if (motorspeedb > maxspeedb) motorspeedb = maxspeedb; // prevent the motor from going beyond max speed
  if (motorspeeda < 60) motorspeeda = 60; // keep the motor speed positive
  if (motorspeedb < 60) motorspeedb = 60; // keep the motor speed positive

  forward_brake(motorspeedb, motorspeeda);
  printSensorValues(); // Print sensor values regardless of the robot's state
  
}

void forward_brake(int posb, int posa) {
  // Set the appropriate values for AIN1, AIN2, BIN1, BIN2 to control motor direction
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, posb);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, posa);

  /*Serial.print(" a:");
  Serial.print(posa);
  Serial.print(" b:");
  Serial.print(posb);
  Serial.println();*/
}

/*void PID_control() {
  uint16_t position = qtr.readLineWhite(sensorValues);
  int error = 3500 - position;
  Serial.println(position);
  P = error;
 
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}*/

void printSensorValues() {
  // Read sensor values into the array
  qtr.read(sensorValues);

  // Print each sensor value
  for (uint8_t i = 0; i < SensorCount; i++) {
  Serial.print(sensorValues[i]);
  Serial.print(' ');
}
Serial.println();
}