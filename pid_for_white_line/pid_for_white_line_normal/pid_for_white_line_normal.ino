#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint16_t threshold = 1950;
uint8_t binaryValues[SensorCount];   // Array to store '0' or '1' based on the threshold

float Kp = 0.04; // Set up the constants value 0.0575
float Ki = 0;
float Kd = 0.06;//0.1
int P;
int I;
int D;
int lastError = 0;
int prePos = 0;
boolean onoff = true;

// Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7.4 V 
const uint8_t maxspeeda = 150;//to make fast put 230
const uint8_t maxspeedb = 150;//to make fast put 230
const uint8_t basespeeda = 100;//to make fast put 230
const uint8_t basespeedb = 100;//to make fast put 230

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
  for (uint16_t i = 0; i < 30; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(startLED,5);
  delay(3000);
  analogWrite(startLED,0);
}

void loop() {
  //unsigned int sensors[8];
  //uint16_t position = qtr.readLineWhite(sensorValues);
  getDigitalSensorValues();
  int linePosition = findLinePosition(binaryValues, SensorCount);
  // Print the line position
  //Serial.print("Line Position: ");
  //Serial.println(linePosition);
  int error = linePosition - 3500;
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
  // Print sensor values regardless of the robot's state
  /*Serial.print(error);
  Serial.println();*/
  printArray(binaryValues, SensorCount);
  //printAnalogSensorValues();

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

void getDigitalSensorValues() {
  // Read sensor values into the array
  qtr.read(sensorValues);

  // Print each sensor value
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < threshold) {
    binaryValues[i] = 1;  // Store '0' if below threshold
    } else {
      binaryValues[i] = 0;  // Store '1' if above threshold
  }
}
return binaryValues ;
}

int findLinePosition(uint8_t binary[], uint8_t count) {
  int weightedSum = 0;       // Sum of weighted indices
  int countOnLine = 0;       // Number of sensors detecting the line
  int currentPos = 0;

  // Iterate through the binary array to calculate the weighted average position
  for (uint8_t i = 0; i < count; i++) {
    if (binary[i] == 1) {     // Check if the sensor detects the line
      weightedSum += i;       // Add the index to the weighted sum
      countOnLine++;          // Increment count of sensors on the line
    }
  }

  // Calculate and return the average position, or -1 if no line is detected
  if (countOnLine > 0) {
    currentPos = weightedSum / countOnLine*1000;
    prePos = currentPos;
    return currentPos;  // Return the average position
  } else {
    return prePos;  // Return -1 if no line is detected
  }
}

void printArray(uint8_t arr[], uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(' ');  // Add a space between elements for readability
  }
  Serial.println();  // Move to the next line after printing the array
}

void printAnalogSensorValues() {
  // Read sensor values into the array
  qtr.read(sensorValues);

  // Print each sensor value
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
}
Serial.println();
}