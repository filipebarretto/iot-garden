
/*
 * 
 * 
 * 
 */

// CONSTANTS
const int RELAY_PIN = 15;
const int MOISTURE_SENSOR_PIN = 15;

const int MOISTURE_THRESHOLD = 0.8;

const int MOISTURE_CHECK_FREQUENCY = 1 * 60 * 1000;   // 1 MINUTE
const int DATABASE_SYNC_FREQUENCY = 5 * 60 * 1000;    // 5 MINUTES 

void setup() {
  Serial.begin(9600);

  // SETUP PINS
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MOISTURE_SENSOR_PIN, INPUT);

}

void loop() {

  // CHECK MOISTURE
  moisture = analogRead(MOISTURE_SENSOR_PIN); 

  Serial.println(moisture);
  
  if (moisture < MOISTURE_THRESHOLD) {
    // WATERING PLANTS
    Serial.println("WATERING PLANTS");
    digitalWrite(RELAY_PIN, LOW);
    delay(1000);
  
    digitalWrite(RELAY_PIN, HIGH);
  }

  delay(1000);
  
}
