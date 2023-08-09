/**
 * Firmware for RP204 running on the rover or base station to support sending/receiving of 
 * correction data via ESP32 radio. This firmware reads data from TinkerCharge and makes it 
 * available to the ESP32 to display on a webpage. The firmware also outputs GNSS serial data
 * to the RP2040 USB serial port.
 * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * This is becuase it uses software serial to communicate to the ESP32. Hardware serial is
 * used for the more time critical GNSS communications.
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

// MAX17055 Battery Fuel Cell Gauge

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
    // Pauses till serial starts. Do not use when running without a computer attached
    // or it will pause indefinetly
    //while (!Serial){};

    // GNSS input/output Serial is Serial1 using default 0,1 (TX, RX) pins
    // Loop through valid baud rates and determine the current setting
    // Set Serial1 to the detected baud rate, stop if a baud rate is not found
    // From NavSpark binary protocol. Search for "SkyTrq Application Note AN0037"
    // Currently available at: https://www.navsparkforum.com.tw/download/file.php?id=1162&sid=dc2418f065ec011e1b27cfa77bf22b19
    if(!detectCurrentBaudRate())
    {
        Serial.println("No valid baud rate found to talk to receiver, stopping");
        while(1);
    }
    
    delay(250);
    // Reset receiver to factory defaults, which are the Rover settings
    uint8_t factory_defaults[] = {0xA0, 0xA1, 0x00, 0x02, 0x04, 0x01, 0x05, 0x0D, 0x0A};
    int msg_size = sizeof(factory_defaults);
    sendMessage(factory_defaults,msg_size);

    // ESP32 serial connection
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

    // Periodically write SOC data to ESP32
    if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
    { 
        Serial.println(millis());
        readAndSendSOC();
        last_soc_time = current_time;
    }

    // Print GNSS output to USB serial
//    while (Serial1.available()) 
//    {
//        Serial.print(Serial1.read());
//    }

}

// Read and send state of charge (SOC) data to ESP32
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

}

// Loop through valid baud rates and determine the current setting
bool detectCurrentBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which buad rate returns a valid
    // ACK message
    int valid_baud_rates[9] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}; 
    
    // A character array for one block of GNSS data
    uint8_t serial_data[2500];

    // How long to wait for response from receiver
    int wait_duration = 250;

    // Loop through possible baud rates
    for (int i=0;i<9;i++)
    {
        // Open the serial connection to the receiver
        Serial1.begin(valid_baud_rates[i]);
        
        // Send a message to request the software version and check if a valid
        // ACK message is received
        uint8_t msg_send[] = {0xA0, 0xA1, 0x00, 0x02, 0x04, 0x01, 0x05, 0x0D, 0x0A};
        Serial.print("Trying baud rate = ");Serial.println(valid_baud_rates[i]);
        int msg_size = sizeof(msg_send);

        sendMessage(msg_send, msg_size);
        
        unsigned long time_start = millis();
        int input_pos = 0;
        bool baud_rate_found = false;
        
        // Wait for a response from the receiver
        while (millis() < time_start + wait_duration && !baud_rate_found)
        {
            //Serial.println(millis());
            if (Serial1.available() && !baud_rate_found)
            {
                // Read data from serial
                serial_data[input_pos] = Serial1.read();
                //Serial.print(serial_data[input_pos], HEX);Serial.print(" ");

                // Start checking last 5 bytes found for a valid ACK message
                if (input_pos >=4)
                {
                    // Valid message was received, we have the right baud rate
                    if (serial_data[input_pos-4] == 0xA0 &&
                        serial_data[input_pos-3] == 0xA1 &&
                        serial_data[input_pos-2] == 0x0 &&
                        serial_data[input_pos-1] == 0x2 &&
                        serial_data[input_pos] == 0x83)
                    {
                        baud_rate_found = true;                    
                    }
                }
                
                input_pos++;
            }
        }

        if (baud_rate_found)
        {
            Serial.print("Found correct baud rate of ");Serial.println(valid_baud_rates[i]);
            return true;
        }
        else
        {
            Serial1.end();
            input_pos = 0;
        }
    }

    return false;
}

void sendMessage(uint8_t *msg_send, int msg_size)
{
    Serial1.write(msg_send,msg_size);
    Serial.print("Sending ");Serial.print(" ");
    for (int i=0;i<msg_size;i++)
    {
        Serial.print(msg_send[i],HEX);Serial.print(" ");
    }
    Serial.println("");
}
