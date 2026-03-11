#pragma once
#include "SPIFFS.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "Flight_Controller.h"
#include "PID_Webserver.h"
#include "PID.h"
class FC;
class PID_Webserver
{
public:    
    PID_Webserver(FC& fcRef, PID& pidRef) : fc(fcRef), pid(pidRef) {}
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
    bool motorsOff();

private: 
    FC& fc;
    PID& pid;
    AsyncWebServer server{80};
    AsyncEventSource events{"/events"};

    const char *ssid = "Untrusted_Network";
    const char *password = "rapidcream878";
};

