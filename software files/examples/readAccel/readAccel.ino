#include "ADXL345.h"

ADXL345 accellerometer;

void setup() {
    Serial.begin(115200); 
    while (!Serial) {
        delay(10); // Waiting for serial to open
    }

    Serial.println("ADXL345 initializing.");

    accellerometer.begin(I2C_MODE);

    delay(10);
    
    // Setting range, datarate, resolution etc.
    accellerometer.setRange(RANGE_16G);
    accellerometer.setDataRate(RATE_100_HZ);
    accellerometer.setResolution(RESOLUTION_13_bit);
    //accellerometer.setJustify(LEFT_JUSTIFY);
    accellerometer.enableFullRes();
    accellerometer.enableMeasurement();
    Serial.println("ADXL345 initiated.");

    delay(200);
}

void loop() {
    float x, y, z;
    accellerometer.readAcceleration(x, y, z); // Reading data from the sensor.
    // Print the read data on the serial monitor
        Serial.print(x);
        Serial.print(";");
        Serial.print(y);
        Serial.print(";");
        Serial.print(z);
        Serial.println(";");

        delay(100);
    } 

