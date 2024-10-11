const uint8_t SensorCount = 8;
int sensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Analog pins where the sensors are connected
uint16_t sensorValues[SensorCount];  // Array to store analog sensor readings
const uint16_t threshold = 500;  // Example threshold value to differentiate black from white
uint8_t binaryValues[SensorCount];  // Array to store '0' or '1' based on the threshold

float Kp = 0.04; // Set up the PID constants
float Ki = 0;
float Kd = 0.06;
int P;
int I;
int D;
int lastError = 0;
int prePos = 0;
boolean onoff = true;

// Motor settings
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

// Motor driver pins for L298N
int ENA = 6;  // Enable pin for Motor A (PWM)
int IN1 = 9;  // Input 1 for Motor A
int IN2 = 8;  // Input 2 for Motor A
int ENB = 3;  // Enable pin for Motor B (PWM)
int IN3 = 5;  // Input 1 for Motor B
int IN4 = 4;  // Input 2 for Motor B

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  forward_brake(0, 0);  // Initial stop
}

void loop() {
  getDigitalSensorValues();  // Get sensor readings and convert them to binary values
  int linePosition = findLinePosition(binaryValues, SensorCount);  // Get line position based on sensor readings

  int error = linePosition - 3500;  // Calculate the error for PID control

  int motorSpeed = Kp * error + Kd * (error - lastError);  // Calculate motor speed based on error
  lastError = error;

  int motorspeeda = basespeeda - motorSpeed;  // Adjust motor speeds
  int motorspeedb = basespeedb + motorSpeed;

  if (motorspeeda > maxspeeda) motorspeeda = maxspeeda;  // Limit max speed
  if (motorspeedb > maxspeedb) motorspeedb = maxspeedb;  // Limit max speed
  if (motorspeeda < 60) motorspeeda = 60;  // Ensure positive motor speed
  if (motorspeedb < 60) motorspeedb = 60;  // Ensure positive motor speed

  forward_brake(motorspeedb, motorspeeda);  // Drive the motors
  printArray(binaryValues, SensorCount);  // Print binary sensor values for debugging
}

// Function to drive motors
void forward_brake(int posb, int posa) {
  // Motor A control
  digitalWrite(IN1, HIGH);  // Forward
  digitalWrite(IN2, LOW);
  analogWrite(ENA, posa);  // Set motor A speed using PWM

  // Motor B control
  digitalWrite(IN3, HIGH);  // Forward
  digitalWrite(IN4, LOW);
  analogWrite(ENB, posb);  // Set motor B speed using PWM
}

// Read analog sensor values and convert to binary
void getDigitalSensorValues() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);  // Read the analog value of each sensor
    if (sensorValues[i] < threshold) {
      binaryValues[i] = 1;  // Line detected
    } else {
      binaryValues[i] = 0;  // No line detected
    }
  }
}

// Function to calculate the line position based on sensor values
int findLinePosition(uint8_t binary[], uint8_t count) {
  int weightedSum = 0;  // Sum of weighted indices
  int countOnLine = 0;  // Number of sensors detecting the line
  int currentPos = 0;

  for (uint8_t i = 0; i < count; i++) {
    if (binary[i] == 1) {
      weightedSum += i * 1000;  // Multiply the index by 1000 for more precision
      countOnLine++;
    }
  }

  if (countOnLine > 0) {
    currentPos = weightedSum / countOnLine;
    prePos = currentPos;  // Store the last known position
    return currentPos;
  } else {
    return prePos;  // Return last known position if no line is detected
  }
}

// Function to print binary sensor values
void printArray(uint8_t arr[], uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(' ');
  }
  Serial.println();
}
