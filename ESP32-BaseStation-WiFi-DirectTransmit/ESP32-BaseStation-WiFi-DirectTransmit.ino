/**
 * Firmware for ESP32 running on the base station to send correction data directly to rover over WiFi network.
 * This firmware connects to a WiFi network (modify inputs.h for your settings) and creates a TCP server, 
 * which the rover can connect to.
 * This firmware also displays basic information the information sent and battery if attched,
 * which is available on the ESP32's IP address (printed to serial at startup).
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */
 
 // Libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SoftwareSerial.h"
#include "SerialTransfer.h"
#include <esp_task_wdt.h>

// Local files
#include "homepage.h"
#include "tinkercharge.h"
#include "rtk.h"
#include "inputs.h"

// ESP32 watch dog timer (s)
#define WDT_TIMEOUT 10

// AsyncWebServer object on port 80
AsyncWebServer server(80);

// Event Source on /events
AsyncEventSource events("/events");

// Communications port to other ESP32
#define COM_PORT 4081

// Server for communicating RTCM data
WiFiServer tcp_server(COM_PORT);

// Max RTCM data length
#define MAX_SERIAL_LENGTH 5000

// Remote client handle
WiFiClient remote_client;

// Software serial connection for TinkerNav data
SoftwareSerial tinkernav_serial;

// Library and structure for transfering data from TinkerNav
SerialTransfer transfer_from_nav;
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
} data_for_tinkersend;

// Period for updating website data
unsigned long next_update = 0;
int update_period = 2000;

// Number of RTCM data packages sent
unsigned long num_rtcm_uploads = 0;

void setup() 
{

    // Enable ESP32 watch dog timer and add the curent thread to it
    // The watch dog timer resets the ESP32 if there are any unhandled
    // exceptions that cause the ESP32 to hang.
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
  
    // USB serial
    Serial.begin(115200);
    // Pauses till serial connection is made, does not work without
    // a computer attached, will pause indefinetly when headless   
    //while (!Serial){};
    Serial.println("Serial Open");

    // Software serial connection to TinkerNav
    tinkernav_serial.begin(9600, SWSERIAL_8N1, 1, 0);
    transfer_from_nav.begin(tinkernav_serial);

    // GNSS hardware serial connection (rx/tx)
    // Receives RTCM correction data from the PX1125R
    Serial1.begin(115200, SERIAL_8N1, 21, 20);

    // Connect to WiFi
    connectWiFi();

    // Home page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle Home Page");
        request->send_P(200, "text/html", home_html);
    });

    // TinkerCharge data page
    server.on("/tinkercharge", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle TinkerCharge");
        request->send_P(200, "text/html", tc_html, initTinkercharge);
    });

    
    // RTK data page
    server.on("/rtk", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle RTK");
        request->send_P(200, "text/html", rtk_html, initRTK);
    });

    // Handle Web Server Events
    events.onConnect([](AsyncEventSourceClient *client)
    {
        if(client->lastId())
        {
          Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
        }
        client->send("hello!", NULL, millis(), 1000);
    });
    server.addHandler(&events);
    server.begin();

    tcp_server.begin();

}

void loop() 
{

    // Tell the watch dog timer the thread is still alive
    // so that the hardware doesn't reset
    esp_task_wdt_reset();

    // Receive serial data from TinkerCharge via RP2040
    if (transfer_from_nav.available())
    {
        uint16_t rec_size = 0;
        rec_size = transfer_from_nav.rxObj(data_for_tinkersend, rec_size);
        Serial.print(millis());Serial.print(" Transfer from RP2040 complete; Voltage = ");Serial.println(data_for_tinkersend.voltage);
    }

    // Check for client connections
    checkForConnections();

    // Read RTCM data and send to client if connected and data is available
    if (remote_client.connected() && Serial1.available())
    {
        Serial.print(millis());Serial.println(" Before readSerialBufferAndSend");
        readSerialBufferAndSend();
        Serial.print(millis());Serial.println(" After readSerialBufferAndSend");

    }

    // Update data on webpage
    if (millis() > next_update) 
    {

        Serial.print(millis());Serial.println(" Start next_update");

        // Check connection to WiFi and reconnect if needed
        connectWiFi();
        
        // Send Events to update webpages
        events.send("ping",NULL,millis());
        events.send(String(data_for_tinkersend.voltage).c_str(),"voltage",millis());
        events.send(String(data_for_tinkersend.avg_voltage).c_str(),"avg_voltage",millis());
        events.send(String(data_for_tinkersend.current).c_str(),"current",millis());
        events.send(String(data_for_tinkersend.avg_current).c_str(),"avg_current",millis());
        events.send(String(data_for_tinkersend.battery_capacity).c_str(),"battery_capacity",millis());
        events.send(String(data_for_tinkersend.SOC).c_str(),"battery_soc",millis());
        events.send(String(data_for_tinkersend.temperature).c_str(),"tc_temp",millis());
        events.send(String(millis()/1000.0/60.0).c_str(),"up_time",millis());
        events.send(String(num_rtcm_uploads).c_str(),"num_uploads",millis());

        next_update = millis() + update_period;

        Serial.print(millis());Serial.println(" End next_update");


    }
}

// Connect to WiFi
void connectWiFi() 
{
    int try_count = 0;
    while ( WiFi.status() != WL_CONNECTED )
    {
        try_count++;
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.begin( ssid, password );
        Serial.print("Connecting to WiFi .");
        if ( try_count == 10 )
        {
          ESP.restart();
        }
        Serial.print('.');
        delay(1000);
        if(WiFi.status() == WL_CONNECTED)
        {
            Serial.print(" IP Address: ");Serial.println(WiFi.localIP());
            Serial.print("The MAC address for this board is: ");Serial.println(WiFi.macAddress());
        }
    }
}
int data_counter = 0;
bool data_available = false;
char rtcm_data[MAX_SERIAL_LENGTH];
    
// Read serial buffer and pack data into an array for sending
void readSerialBufferAndSend()
{
  
    data_counter = 0;
    data_available = false;

    // Position through current packet
    unsigned long last_read_time = 0;

    // Read a burst of GNSS data, the PX11XX receivers send RTCM
    // data in bursts of messages with hundreds of milliseconds
    // between each burst
    last_read_time = millis();
    while (Serial1.available() || (millis() - last_read_time) < 50)
    {
        if (Serial1.available() && data_counter<MAX_SERIAL_LENGTH)
        {
            // Read data from GNSS via serial
            rtcm_data[data_counter] = Serial1.read();
            data_counter++;
            data_available = true;
            last_read_time = millis();
        }
    }
    
    // Send data to TCP server
    if (data_available)
    {
//        Serial.println(data_counter);
//        if (data_counter < 1000)
//        {
            remote_client.write((uint8_t*)rtcm_data,data_counter);
            Serial.print(millis());Serial.print(" Sent RTCM data length ");Serial.println(data_counter);
            data_available = false;
            num_rtcm_uploads += 1;       
            data_counter = 0;
//        }
    }
}

// Check for client connection requests
void checkForConnections()
{

    if (!remote_client || !remote_client.connected()) 
    {
        //Serial.print(millis());Serial.println(" Checking for new client");
        // Check if a client has connected
        remote_client = tcp_server.available();
        if (remote_client) 
        {
          Serial.print(millis());Serial.println( "New client connected");
        }
    }
}

// Populates initial webpage values on first load
String initTinkercharge(const String& var)
{

    if(var == "VOLTAGE")
    {
      return String(data_for_tinkersend.voltage);
    }
    else if(var == "AVG_VOLTAGE")
    {
      return String(data_for_tinkersend.avg_voltage);
    }
    else if(var == "CURRENT")
    {
      return String(data_for_tinkersend.current);
    }
    else if(var == "AVG_CURRENT")
    {
      return String(data_for_tinkersend.avg_current);
    }
    else if(var == "BATT_CAPACITY")
    {
      return String(data_for_tinkersend.battery_capacity);
    }
    else if(var == "BATT_CHARGE")
    {
      return String(data_for_tinkersend.SOC);
    }
    else if(var == "TEMPERATURE")
    {
      return String(data_for_tinkersend.temperature);
    }
    else if(var == "UP_TIME")
    {
      return String(millis()/1000.0/60);
    }

    return String();
}

// Populates initial webpage values on first load
String initRTK(const String& var)
{

    if(var == "UP_TIME")
    {
      return String(millis()/1000.0/60);
    }
    else if(var == "NUM_UPLOADS")
    {
      return String(num_rtcm_uploads);
    }

    return String();
}
