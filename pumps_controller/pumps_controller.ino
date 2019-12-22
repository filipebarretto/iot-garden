
/*
 * 
 * 
 * 
 */

// CONSTANTS
const int RELAY_PIN = 15;
const int MOISTURE_SENSOR_PIN = 4;

const int MOISTURE_THRESHOLD = 30;

const int MOISTURE_CHECK_FREQUENCY = 10 * 60 * 1000;   // 10 MINUTES
//const int DATABASE_SYNC_FREQUENCY = 5 * 60 * 1000;    // 5 MINUTES 
const int WATER_PLANTS_DURATION = 800;    // 1 SECOND 


float moisture;

void setup() {
  Serial.begin(9600);

  // SETUP PINS
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

}


/*
 * 
 * PAYLOAD:
 * - TIMESTAMP
 * - STRING SENSOR_ID
 * - FLOAT MOISTURE
 * - BOOL WATER_PLANTS
 * 
 */

void loop() {

  // CHECK MOISTURE
  moisture = (4096 - analogRead(MOISTURE_SENSOR_PIN)) / 4096.0 * 100.0; 

  Serial.println("Soil Moisture: ");
  Serial.println(moisture);

  // SEND DATA TO IOT CORE AND DYNAMODB DATABASE
  
  if (moisture < MOISTURE_THRESHOLD) {
    // WATERING PLANTS
    Serial.println("WATERING PLANTS");
    
    digitalWrite(RELAY_PIN, LOW);
    delay(WATER_PLANTS_DURATION);  
    digitalWrite(RELAY_PIN, HIGH);
    
  } else {
    Serial.println("NOT WATERING PLANTS");
  }
  

  delay(10000);
  
}
