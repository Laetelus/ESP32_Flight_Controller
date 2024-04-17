#ifndef PID_WEBSERVER
#define PID_WEBSERVER


#include "Flight_Controller.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

struct PID_Webserver 
{
  
    void initSPIFFS();
    void Wifi_task();
    void initWiFi();
    void disconnect_wifi();
    void Handle_Server();
    void handleGetPID(AsyncWebServerRequest *request);
    void handleSetPID(AsyncWebServerRequest *request);
    void checkWiFiConnection();
    void fillPIDJson(DynamicJsonDocument &doc);
    String updatePIDFromRequest(AsyncWebServerRequest *request);
    bool savePIDValues();
    bool loadPIDValues();
    
    AsyncWebServer server{80};
    AsyncEventSource events{"/events"};

    const char *ssid = "Untrusted_Network";
    const char *password = "rapidcream878";
}; 

// extern Flight_Controller flightController;

#endif