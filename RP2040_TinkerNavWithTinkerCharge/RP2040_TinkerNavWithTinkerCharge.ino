/**
 * Firmware for RP204 running on the rover or base station to support sending/receiving of 
 * correction data via ESP32 radio. This firmware reads data from TinkerCharge and makes it 
 * available to the ESP32 to display on a webpage. The firmware also outputs GNSS serial data
 * to the RP2040 USB serial port.
 * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * This is becuase it uses software serial to communicate to the ESP32. Hardware serial is
 * used more critical GNSS communications.
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

// !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!

#include "Arduino.h"
#include <Wire.h>
#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include "pico/stdlib.h"
#include <MAX17055_TR.h>

// Serial connection to ESP32 radio (RX, TX)
SoftwareSerial swSerial(20, 3);

// Library and structure for transfering data to TinkerSend radio
SerialTransfer radioTransfer;
struct STRUCT 
{
    float voltage;
    float avg_voltage;
    float current;
    float avg_current;
    float battery_capacity;
    float battery_age;
    float cycle_counter;
    float SOC;
    float temperature;
} dataForTinkerSend;

// XXX MAX17055 Battery Fuel Cell Gauge XXX

// I2C pins
#define SDA 26
#define SCL 27

MAX17055 max17055;

// Timer to write SOC data on a specified periodic (ms)
unsigned long last_soc_time = 0;
int soc_periodic = 2000;

void setup() 
{

    // Pico USB Serial
    Serial.begin(115200);
    // Waits for computer connection, do not use if not running with computer
    //while (!Serial){};

    // GNSS input/output
    // Uses default 0,1 (TX, RX) pins
    Serial1.begin(115200);

    // Radio serial connection
    swSerial.begin(9600);
    radioTransfer.begin(swSerial);

    // Set I2C pins for communicating with MAX17055
    Wire1.setSDA(SDA);
    Wire1.setSCL(SCL);
    Wire1.begin();

    // Configure MAX17055
    max17055.setResistSensor(0.01); 
    max17055.setCapacity(4400);
    max17055.setChargeTermination(44);
    max17055.setEmptyVoltage(3.3);

    last_soc_time = millis();

    Serial.println("Setup Complete");

}

void loop() 
{

    unsigned long current_time = millis();
    //Serial.println(current_time);

    // Periodically write SOC data to TinkerSend WiFi
    if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
    { 
        Serial.println(millis());
        readAndSendSOC();
        last_soc_time = current_time;
    }

    // Print GNSS output to USB serial
    while (Serial1.available()) 
    {
        Serial.print(Serial1.read(),HEX);
    }

}

// Read and send state of charge (SOC) data to TinkerSend WiFi
void readAndSendSOC()
{
    // Read data and pack into structure
    dataForTinkerSend.voltage = max17055.getInstantaneousVoltage();
    dataForTinkerSend.avg_voltage = max17055.getAvgVoltage();
    dataForTinkerSend.current = max17055.getInstantaneousCurrent();
    dataForTinkerSend.avg_current = max17055.getAvgCurrent();
    dataForTinkerSend.battery_capacity = max17055.getCalculatedCapacity();
    dataForTinkerSend.battery_age = max17055.getBatteryAge();
    dataForTinkerSend.cycle_counter = max17055.getChargeCycle();
    dataForTinkerSend.SOC = max17055.getSOC();
    dataForTinkerSend.temperature = max17055.getTemp();

    Serial.println("");
    Serial.print("Voltage: ");Serial.println(dataForTinkerSend.voltage);
    Serial.print("Avg Voltage: ");Serial.println(dataForTinkerSend.avg_voltage);
    Serial.print("Current: ");Serial.println(dataForTinkerSend.current);
    Serial.print("Avg Current: ");Serial.println(dataForTinkerSend.avg_current);
    Serial.print("Battery Capactity: ");Serial.println(dataForTinkerSend.battery_capacity);
    Serial.print("Battery Age: ");Serial.println(dataForTinkerSend.battery_age);
    Serial.print("Number of Cycles: ");Serial.println(dataForTinkerSend.cycle_counter);
    Serial.print("SOC: ");Serial.println(dataForTinkerSend.SOC);
    Serial.print("Temperature: ");Serial.println(dataForTinkerSend.temperature);
    Serial.println("------------------------------------------------------------");
    
    uint16_t sendSize = 0;

    // Send data to TinkerSend radio using serial connection
    sendSize = radioTransfer.txObj(dataForTinkerSend,sendSize);
    radioTransfer.sendData(sendSize);
    //swSerial.write("hello world");
    //Serial.println("sending hello world");
    //Serial.print("send size: ");Serial.println(sendSize);
}
