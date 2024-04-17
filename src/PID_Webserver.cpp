#include <WiFi.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include "SPIFFS.h"
#include "FS.h"
#include "Flight_Controller.h"
#include "PID_Webserver.h"

extern struct Flight_Controller flightController;

PID_Webserver w; 

void WiFiTask(void *parameter) {
  for(;;) { // Infinite loop
    if (flightController.areMotorsOff()) {
      w.initWiFi();
      w.checkWiFiConnection();
    } else {
      w.disconnect_wifi();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to prevent the task from using all CPU time
  }
}

void PID_Webserver::Wifi_task(){
    
      // Create a task for WiFi management
  xTaskCreatePinnedToCore(
    WiFiTask, /* Task function */
    "WiFiTask", /* Name of task */
    10000, /* Stack size of task */
    NULL, /* Parameter of the task */
    1, /* Priority of the task */
    NULL, /* Task handle to keep track of created task */
    0); /* Core where the task should run */
}


void PID_Webserver::initSPIFFS()
{
    if (!SPIFFS.begin())
    {
        Serial.println("An error has occurred while mounting SPIFFS");
    }
    else
    {
        Serial.println("SPIFFS mounted successfully");
    }
}

void PID_Webserver::initWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        // Serial.println("\nConnecting to WiFi...");
    }
}

void PID_Webserver::disconnect_wifi()
{
    // Serial.println("Disconnecting WiFi because motors are on.");
    WiFi.disconnect(true); // Disconnect WiFi and erase credentials
    WiFi.mode(WIFI_OFF);   // Turn off WiFi

    while (WiFi.status() == WL_CONNECTED)
    {
        Serial.print('.');
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void PID_Webserver::checkWiFiConnection()
{
    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
        static bool isConnected = false;
        if (!isConnected)
        {
            Serial.println("Connected to WiFi");
            Serial.println(WiFi.localIP());
            isConnected = true;
            Handle_Server();
        }
    }
    else
    {
        Serial.print(".");
    }
}

void PID_Webserver::Handle_Server()
{
    // Serve the main index page from SPIFFS
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/getPID", HTTP_GET, [this](AsyncWebServerRequest *request)
              { handleGetPID(request); });

    server.on("/setPID", HTTP_POST, [this](AsyncWebServerRequest *request)
              { handleSetPID(request); });

    events.onConnect([](AsyncEventSourceClient *client)
                     {
        if (client->lastId()) {
            Serial.printf("Client reconnected! Last message ID: %u\n", client->lastId());
        }
        client->send("hello!", NULL, millis(), 10000); });

    server.addHandler(&events);
    server.begin();
}

bool PID_Webserver::savePIDValues()
{
    File file = SPIFFS.open("/pid_values.txt", FILE_WRITE);
    if (!file)
    {
        Serial.println("There was an error opening the file for writing");
        return false;
    }

    if (flightController.areMotorsOff())
    {
        // Only write roll and yaw values, since pitch will mirror roll
        file.printf("P_GAIN_ROLL:%f\n", flightController.pid_p_gain_roll);
        file.printf("I_GAIN_ROLL:%f\n", flightController.pid_i_gain_roll);
        file.printf("D_GAIN_ROLL:%f\n", flightController.pid_d_gain_roll);

        file.printf("P_GAIN_YAW:%f\n", flightController.pid_p_gain_yaw);
        file.printf("I_GAIN_YAW:%f\n", flightController.pid_i_gain_yaw);
        file.printf("D_GAIN_YAW:%f\n", flightController.pid_d_gain_yaw);
    }

    file.close();
    return true;
}

bool PID_Webserver::loadPIDValues()
{
    File file = SPIFFS.open("/pid_values.txt", FILE_READ);
    if (!file)
    {
        Serial.println("There was an error opening the file for reading");
        return false;
    }

    String line;
    while (file.available())
    {
        line = file.readStringUntil('\n');
        if (line.startsWith("P_GAIN_ROLL:"))
        {
            flightController.pid_p_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            flightController.pid_p_gain_pitch = flightController.pid_p_gain_roll;
        }
        else if (line.startsWith("I_GAIN_ROLL:"))
        {
            flightController.pid_i_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            flightController.pid_i_gain_pitch = flightController.pid_i_gain_roll;
        }
        else if (line.startsWith("D_GAIN_ROLL:"))
        {
            flightController.pid_d_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            flightController.pid_d_gain_pitch = flightController.pid_d_gain_roll;
        }
        else if (line.startsWith("P_GAIN_YAW:"))
        {
            flightController.pid_p_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("I_GAIN_YAW:"))
        {
           flightController.pid_i_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("D_GAIN_YAW:"))
        {
            flightController.pid_d_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
    }

    file.close();
    return true;
}

String formatFloat(float value, unsigned int maxDecimals) {
    char buffer[32]; // Make sure the buffer is large enough to hold the largest possible number
    // Create a format string for snprintf that specifies the maximum number of decimal places
    snprintf(buffer, sizeof(buffer), "%.*f", maxDecimals, value);
    
    // Trim trailing zeros and the decimal point if not needed
    char* end = buffer + strlen(buffer) - 1;
    while (end > buffer && *end == '0') --end;
    if (end > buffer && *end == '.') --end;
    *(end + 1) = '\0'; // Null-terminate the string

    return String(buffer);
}
 
void PID_Webserver::fillPIDJson(DynamicJsonDocument &doc) {
    // Roll PID parameters
    doc["pid_p_gain_roll"] = formatFloat(flightController.pid_p_gain_roll, 5);
    doc["pid_i_gain_roll"] = formatFloat(flightController.pid_i_gain_roll, 5);
    doc["pid_d_gain_roll"] = formatFloat(flightController.pid_d_gain_roll, 5);
    doc["pid_max_roll"] = formatFloat(flightController.pid_max_roll, 5);

    // Pitch PID parameters
    doc["pid_p_gain_pitch"] = formatFloat(flightController.pid_p_gain_pitch, 5);
    doc["pid_i_gain_pitch"] = formatFloat(flightController.pid_i_gain_pitch, 5);
    doc["pid_d_gain_pitch"] = formatFloat(flightController.pid_d_gain_pitch, 5);
    doc["pid_max_pitch"] = formatFloat(flightController.pid_max_pitch, 5);

    // Yaw PID parameters
    doc["pid_p_gain_yaw"] = formatFloat(flightController.pid_p_gain_yaw, 5);
    doc["pid_i_gain_yaw"] = formatFloat(flightController.pid_i_gain_yaw, 5);
    doc["pid_d_gain_yaw"] = formatFloat(flightController.pid_d_gain_yaw, 5);
    doc["pid_max_yaw"] = formatFloat(flightController.pid_max_yaw, 5);
}

String PID_Webserver::updatePIDFromRequest(AsyncWebServerRequest *request)
{
    String response = "";

    // Helper lambda for updating float parameters
    auto updateParamFloat = [&request, &response](const char *name, float &param)
    {
        if (request->hasParam(name, true))
        {
            param = request->getParam(name, true)->value().toFloat();
            response += String(name) + " updated to " + String(param) + ". ";
        }
    };

    // Update Roll PID parameters
    updateParamFloat("pid_p_gain_roll", flightController.pid_p_gain_roll);
    updateParamFloat("pid_i_gain_roll", flightController.pid_i_gain_roll);
    updateParamFloat("pid_d_gain_roll", flightController.pid_d_gain_roll);

    // Assign roll PID values to pitch and yaw as well
    flightController.pid_p_gain_pitch = flightController.pid_p_gain_roll;
    flightController.pid_i_gain_pitch = flightController.pid_i_gain_roll;
    flightController.pid_d_gain_pitch = flightController.pid_d_gain_roll;

    flightController.pid_p_gain_yaw = flightController.pid_p_gain_roll;
    flightController.pid_i_gain_yaw = flightController.pid_i_gain_roll;
    flightController.pid_d_gain_yaw = flightController.pid_d_gain_roll;

    // Update Yaw PID parameters
    updateParamFloat("pid_p_gain_yaw", flightController.pid_p_gain_yaw);
    updateParamFloat("pid_i_gain_yaw", flightController.pid_i_gain_yaw);
    updateParamFloat("pid_d_gain_yaw", flightController.pid_d_gain_yaw);

    if (response.isEmpty())
    {
        response = "No parameters updated.";
    }

    return response;
}

void PID_Webserver::handleSetPID(AsyncWebServerRequest *request)
{
    if (flightController.areMotorsOff())
    {
        String response = updatePIDFromRequest(request);
        if (!response.isEmpty())
        {
            if (savePIDValues())
            {
                request->send(200, "text/plain", "PID values updated and saved.");
            }
            else
            {
                request->send(500, "text/plain", "Failed to save PID values.");
            }
        }
        else
        {
            request->send(400, "text/plain", "No parameters updated.");
        }
    }
    else
    {
        // If motors are running, do not update PID and inform the user
        request->send(503, "text/plain", "Motors are running. Cannot set PID values.");
    }
}

void PID_Webserver::handleGetPID(AsyncWebServerRequest *request)
{
    if (flightController.areMotorsOff())
    {
        DynamicJsonDocument doc(1024);
        fillPIDJson(doc);
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        request->send(200, "application/json", jsonResponse);
    }
    else
    {
        request->send(503, "text/plain", "Motors are running. Cannot retrieve PID values.");
    }
}
