/*const int trigPinA = 9;
const int echoPinA = 10;
const int trigPinB = 5;
const int echoPinB = 6;*/

#define trigPinA 9
#define echoPinA 10
#define trigPinB 5
#define echoPinB 6
int trigPin;
int echoPin;
float distance_A = 0;
float distance_B = 0;
int cur_status_A = 0;
int pre_status_A = 1;
int cur_status_B = 0;

String sensor = ""; // Assign an empty string
bool portal_opened = false;
bool can_go = false;

void setup() {
  pinMode(trigPinA, OUTPUT); // Trig pin as output
  pinMode(echoPinA, INPUT);  // Echo pin as input
  pinMode(trigPinB, OUTPUT); // Trig pin as output
  pinMode(echoPinB, INPUT);  // Echo pin as input
  Serial.begin(9600);       // Initialize serial communication
}

void loop() {
  distance_A = distance_measuring("A");
  distance_B = distance_measuring("B");

  if (distance_A > 12.50) {cur_status_A = 1; }
  else{cur_status_A = 0; }

  if (distance_B > 12.50) {cur_status_B = 1; }
  else{cur_status_B = 0; }

  if (cur_status_A == 1 && pre_status_A == 0) {
    portal_opened = true;
  } 
  Serial.println(portal_opened);

  if (portal_opened && cur_status_B == 1) {
    can_go = true;
  } else {
    can_go = false;
  }

  if (can_go) {
    Serial.println("Gooooo.....");
    delay(5000);
    portal_opened = false;
  } else {
    Serial.println("Can not go.");
    if (cur_status_A == 0) {
      portal_opened = false;
    }
  }

pre_status_A = cur_status_A;
}

float distance_measuring(String sensor) {
  float totalDistance = 0;
  float averageDistance = 0;
  int numReadings = 10;

  for (int i = 0; i < numReadings; i++) {
    long duration;
    float distance;
    if (sensor == "A") {
    trigPin = trigPinA;
    echoPin = echoPinA;
  } else {
    trigPin = trigPinB;
    echoPin = echoPinB;
  }
    // Send a 10 microsecond HIGH pulse to the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin and measure how long it is HIGH
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in centimeters
    distance = (duration * 0.034) / 2;

    // Add the distance to the total
    totalDistance += distance;

    // Short delay between readings
    delay(30); // 50 ms delay between readings
  }

  // Calculate the average distance
  averageDistance = totalDistance / numReadings;
  return averageDistance;

  // Print the average distance
  /*Serial.print("Distance ");
  Serial.print(sensor + ": ");
  Serial.print(averageDistance);
  Serial.println(" cm");*/ 

  delay(50); // Wait for a second before starting the next set of reading
}
