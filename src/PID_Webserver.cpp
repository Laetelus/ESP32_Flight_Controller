#include <WiFi.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include "SPIFFS.h"
#include "FS.h"
#include "Flight_Controller.h"

void Flight_Controller::initSPIFFS()
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

void Flight_Controller::initWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        // Serial.println("\nConnecting to WiFi...");
    }
}

void Flight_Controller::disconnect_wifi()
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

void Flight_Controller::checkWiFiConnection()
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

void Flight_Controller::Handle_Server()
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

bool Flight_Controller::savePIDValues()
{
    File file = SPIFFS.open("/pid_values.txt", FILE_WRITE);
    if (!file)
    {
        Serial.println("There was an error opening the file for writing");
        return false;
    }

    if (areMotorsOff())
    {
        // Only write roll and yaw values, since pitch will mirror roll
        file.printf("P_GAIN_ROLL:%f\n", pid_p_gain_roll);
        file.printf("I_GAIN_ROLL:%f\n", pid_i_gain_roll);
        file.printf("D_GAIN_ROLL:%f\n", pid_d_gain_roll);

        file.printf("P_GAIN_YAW:%f\n", pid_p_gain_yaw);
        file.printf("I_GAIN_YAW:%f\n", pid_i_gain_yaw);
        file.printf("D_GAIN_YAW:%f\n", pid_d_gain_yaw);
    }

    file.close();
    return true;
}

bool Flight_Controller::loadPIDValues()
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
            pid_p_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            pid_p_gain_pitch = pid_p_gain_roll;
        }
        else if (line.startsWith("I_GAIN_ROLL:"))
        {
            pid_i_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            pid_i_gain_pitch = pid_i_gain_roll;
        }
        else if (line.startsWith("D_GAIN_ROLL:"))
        {
            pid_d_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            pid_d_gain_pitch = pid_d_gain_roll;
        }
        else if (line.startsWith("P_GAIN_YAW:"))
        {
            pid_p_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("I_GAIN_YAW:"))
        {
            pid_i_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("D_GAIN_YAW:"))
        {
            pid_d_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
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
 
void Flight_Controller::fillPIDJson(DynamicJsonDocument &doc) {
    // Roll PID parameters
    doc["pid_p_gain_roll"] = formatFloat(pid_p_gain_roll, 5);
    doc["pid_i_gain_roll"] = formatFloat(pid_i_gain_roll, 5);
    doc["pid_d_gain_roll"] = formatFloat(pid_d_gain_roll, 5);
    doc["pid_max_roll"] = formatFloat(pid_max_roll, 5);

    // Pitch PID parameters
    doc["pid_p_gain_pitch"] = formatFloat(pid_p_gain_pitch, 5);
    doc["pid_i_gain_pitch"] = formatFloat(pid_i_gain_pitch, 5);
    doc["pid_d_gain_pitch"] = formatFloat(pid_d_gain_pitch, 5);
    doc["pid_max_pitch"] = formatFloat(pid_max_pitch, 5);

    // Yaw PID parameters
    doc["pid_p_gain_yaw"] = formatFloat(pid_p_gain_yaw, 5);
    doc["pid_i_gain_yaw"] = formatFloat(pid_i_gain_yaw, 5);
    doc["pid_d_gain_yaw"] = formatFloat(pid_d_gain_yaw, 5);
    doc["pid_max_yaw"] = formatFloat(pid_max_yaw, 5);
}

String Flight_Controller::updatePIDFromRequest(AsyncWebServerRequest *request)
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
    updateParamFloat("pid_p_gain_roll", pid_p_gain_roll);
    updateParamFloat("pid_i_gain_roll", pid_i_gain_roll);
    updateParamFloat("pid_d_gain_roll", pid_d_gain_roll);

    // Assign roll PID values to pitch and yaw as well
    pid_p_gain_pitch = pid_p_gain_roll;
    pid_i_gain_pitch = pid_i_gain_roll;
    pid_d_gain_pitch = pid_d_gain_roll;

    pid_p_gain_yaw = pid_p_gain_roll;
    pid_i_gain_yaw = pid_i_gain_roll;
    pid_d_gain_yaw = pid_d_gain_roll;

    // Update Yaw PID parameters
    updateParamFloat("pid_p_gain_yaw", pid_p_gain_yaw);
    updateParamFloat("pid_i_gain_yaw", pid_i_gain_yaw);
    updateParamFloat("pid_d_gain_yaw", pid_d_gain_yaw);

    if (response.isEmpty())
    {
        response = "No parameters updated.";
    }

    return response;
}

void Flight_Controller::handleSetPID(AsyncWebServerRequest *request)
{
    if (areMotorsOff())
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

void Flight_Controller::handleGetPID(AsyncWebServerRequest *request)
{
    if (areMotorsOff())
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
