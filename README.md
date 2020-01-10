# IoT Plant Watering

### About

IoT Solution using ESP32 with moisture sensor and water pumps to keep plants healthy, sending data to the Cloud for statistics and logs. In a defined period, the moisture sensor detects the soil moisture and sends the data to ESP32, that decides if it should turn on the pump to water the soil or not, and sends the information including the moisture, pump_id and if it was watered or not to IoT Core, that saves it in DynamoDB.


### Architecture

![IoT Plant Watering Architecture](https://github.com/filipebarretto/iot-plant-watering/blob/master/project-images/iot-plant-watering-achitecture.png?raw=true)

