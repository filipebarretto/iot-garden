
/*
 * 
 * 
 * 
 */

#include "WiFi.h"
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "wifi_config.h"
#include "credentials.h"

#include "esp32-hal-adc.h" // needed for adc pin reset
#include "soc/sens_reg.h" // needed for adc pin reset
uint64_t reg_b; // Used to store Pin registers


// MQTT SETTINGS
const int MQTT_PORT = 8883;
const char MQTT_SUB_TOPIC[] = "iot_garden_controller_confirmation";

WiFiClientSecure net;
PubSubClient client(net);
StaticJsonDocument<200> json;
char data[256];

// CONSTANTS
//const int RELAY_PIN = 15;

// PIN SETUP
const int MOISTURE_SENSOR_PIN_01 = 34;
const int MOISTURE_SENSOR_PIN_02 = 35;
const int MOISTURE_SENSOR_PIN_03 = 32;
const int MOISTURE_SENSOR_PIN_04 = 33;
const int MOISTURE_SENSOR_PIN_05 = 25;
const int MOISTURE_SENSOR_PIN_06 = 26;
const int MOISTURE_SENSOR_PIN_07 = 27;

const int TEMPERATURE_AND_HUMIDITY_SENSOR_PIN = 14;

//const int MOISTURE_THRESHOLD = 30;

//const int LOOP_PERIOD = 1 * 60 * 1000;   // 1 MINUTE
const int LOOP_PERIOD = 30 * 1000;   // 30 SECONDS
//const int DATABASE_SYNC_FREQUENCY = 5 * 60 * 1000;    // 5 MINUTES 
//const int WATER_PLANTS_DURATION = 800;    // 1 SECOND 


// SOIL HUMIDITY
float moisture_01;
float moisture_02;
float moisture_03;
float moisture_04;
float moisture_05;
float moisture_06;
float moisture_07;

float air_humidity;
float air_temperature;

String humidity_str;
String temperature_str;

// TEMPERATURE AND HUMIDITY
byte dat[5];


// TEMPERATURE AND HUMIDITY
byte read_temperature_and_humidity_data() {
  byte i = 0;
  byte result = 0;
  for (i = 0; i < 8; i++) {
      while (digitalRead(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN) == LOW); // wait 50us
      delayMicroseconds(30); //The duration of the high level is judged to determine whether the data is '0' or '1'
      if (digitalRead(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN) == HIGH)
        result |= (1 << (8 - i)); //High in the former, low in the post
    while (digitalRead(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN) == HIGH); //Data '1', waiting for the next bit of reception
    }
  return result;
}

void prepare_temperature_and_humidity_read() {
  digitalWrite(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, LOW); //Pull down the bus to send the start signal
  delay(30); //The delay is greater than 18 ms so that DHT 11 can detect the start signal
  digitalWrite(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, HIGH);
  delayMicroseconds(40); //Wait for DHT11 to respond
  pinMode(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, INPUT);
  while(digitalRead(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN) == HIGH);
  delayMicroseconds(80); //The DHT11 responds by pulling the bus low for 80us;
  
  if(digitalRead(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN) == LOW)
    delayMicroseconds(80); //DHT11 pulled up after the bus 80us to start sending data;
  for(int i = 0; i < 5; i++) //Receiving temperature and humidity data, check bits are not considered;
    dat[i] = read_temperature_and_humidity_data();
  pinMode(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, OUTPUT);
  digitalWrite(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, HIGH); //After the completion of a release of data bus, waiting for the host to start the next signal
}


/*
 * START WIFI AND MQTT CONNECTION CONFIGURATION
 */

void pubSubErr(int8_t MQTTErr) {
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connect_to_mqtt(bool nonBlocking = false) {
  Serial.print("MQTT connecting ");
  while (!client.connected()) {
    if (client.connect(THINGNAME)) {
      Serial.println("connected!");
      if (!client.subscribe(MQTT_SUB_TOPIC))
        pubSubErr(client.state());
    }
    else {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking) {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void messageReceived(char *topic, byte *payload, unsigned int length) {
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");

  char inData[length];
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    inData[i] = (char)payload[i];
  }
  Serial.println();

  DeserializationError error = deserializeJson(json, inData);

  const char* command = json["command"];
  Serial.println(command);
  //serve_drink(recipe);
}


void connect_to_wifi() {
  while (WiFi.status() != WL_CONNECTED)  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected"); 
  Serial.print("My IP address is: ");
  Serial.println(WiFi.localIP());

}

void checkWiFiThenMQTT(void) {
  Serial.println("Checking WiFi...");
  connect_to_wifi();
  connect_to_mqtt();
}

/*
 * END WIFI AND MQTT CONNECTION CONFIGURATION
 */




void setup() {
  Serial.begin(9600);

  Serial.println("Setting up NodeMCU...");

  Serial.println("Setting up pins...");
  pinMode(TEMPERATURE_AND_HUMIDITY_SENSOR_PIN, OUTPUT);

  reg_b = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG);

  Serial.println("Setting up connection...");
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass); 
  WiFi.setHostname("iot_plant_esp32_01");
  
  connect_to_wifi();

  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);
  
  connect_to_mqtt();

  // TODO: SETUP PUMP MAPPING
  Serial.println("Setup Finished.");

  // SETUP PINS
  //pinMode(RELAY_PIN, OUTPUT);
  //digitalWrite(RELAY_PIN, HIGH);

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

  Serial.println("Gathering sensor data.");

  // ADC Pin Reset: Do this before every analogRead()
  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);

  // CHECK TEMPERATURE AND HUMIDITY
  prepare_temperature_and_humidity_read();
//  Serial.print("Humidity = ");
  humidity_str = String(dat[0], DEC) + "." + String(dat[1], DEC);
  air_humidity = (humidity_str).toFloat();
//  Serial.print(dat[0], DEC); //Displays the integer bits of humidity;
//  Serial.print('.');
//  Serial.println(dat[1], DEC); //Displays the decimal places of the humidity;
//  Serial.println(air_humidity);
//  Serial.println('%');
//  
//  Serial.print("Temperature = ");
  temperature_str = String(dat[2], DEC) + "." + String(dat[3], DEC);
  air_temperature = (temperature_str).toFloat();
//  Serial.print(dat[2], DEC); //Displays the integer bits of temperature;
//  Serial.print('.');
//  Serial.println(dat[3], DEC); //Displays the decimal places of the temperature;
//  Serial.println(air_temperature);
//  Serial.println('C');


//  byte checksum = dat[0] + dat[1] + dat[2] + dat[3];
//  if (dat[4] != checksum) 
//    Serial.println("-- Checksum Error!");
//  else
//    Serial.println("-- OK");

  // CHECK SOIL MOISTURE

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_01 = (4096 - analogRead(MOISTURE_SENSOR_PIN_01)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_02 = (4096 - analogRead(MOISTURE_SENSOR_PIN_02)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_03 = (4096 - analogRead(MOISTURE_SENSOR_PIN_03)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_04 = (4096 - analogRead(MOISTURE_SENSOR_PIN_04)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_05 = (4096 - analogRead(MOISTURE_SENSOR_PIN_05)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_06 = (4096 - analogRead(MOISTURE_SENSOR_PIN_06)) / 4096.0 * 100.0;

  WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
  moisture_07 = (4096 - analogRead(MOISTURE_SENSOR_PIN_07)) / 4096.0 * 100.0;

  // CHECK IF DEVICE IS CONNECTED
  if (!client.connected()) {
    checkWiFiThenMQTT();
  } else {
    client.loop();  
  }

  // PREPARE DATA PAYLOAD TO SEND TO MQTT TOPIC
  String payload = "{\"m_01\": " + String(moisture_01) + ", \"m_02\": " + String(moisture_02) + ", \"m_03\": " + String(moisture_03) + ", \"m_04\": " + String(moisture_04) + ", \"m_05\": " + String(moisture_05) +  ", \"m_06\": " + String(moisture_06) +  ", \"m_07\": " + String(moisture_07) +  ", \"t\": " + String(air_temperature) +  ", \"h\": " + String(air_humidity) + "}";
  payload.toCharArray(data, (payload.length() + 1));
  Serial.println(payload);
  client.publish("iot_garden_controller", data);

  

//  Serial.println("Soil Moisture 01: ");
//  Serial.println(moisture_01);
//  Serial.println("Soil Moisture 02: ");
//  Serial.println(moisture_02);
//  Serial.println("Soil Moisture 03: ");
//  Serial.println(moisture_03);
//  Serial.println("Soil Moisture 04: ");
//  Serial.println(moisture_04);
//  Serial.println("Soil Moisture 05: ");
//  Serial.println(moisture_05);
//  Serial.println("Soil Moisture 06: ");
//  Serial.println(moisture_06);
//  Serial.println("Soil Moisture 07: ");
//  Serial.println(moisture_07);

  // SEND DATA TO IOT CORE AND DYNAMODB DATABASE

  /*
  if (moisture < MOISTURE_THRESHOLD) {
    // WATERING PLANTS
    Serial.println("WATERING PLANTS");
    
    digitalWrite(RELAY_PIN, LOW);
    delay(WATER_PLANTS_DURATION);  
    digitalWrite(RELAY_PIN, HIGH);
    
  } else {
    Serial.println("NOT WATERING PLANTS");
  }
  */

  delay(LOOP_PERIOD);
  
}
