#include <WiFi.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include "SPIFFS.h"
#include "FS.h"
#include "Flight_Controller.h"
#include "PID_Webserver.h"
#include "PID.h"

bool PID_Webserver::motorsOff()
{
    return fc.Motorstate() == OFF;
}

void WiFiTask(void *parameter)
{
    
    PID_Webserver* ws = static_cast<PID_Webserver*>(parameter);

    for (;;)
    { // Infinite loop
        if (ws->motorsOff())
        {
            ws->initWiFi();
            ws->checkWiFiConnection();
        }
        else
        {
            ws->disconnect_wifi();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

void PID_Webserver::Wifi_task()
{
    xTaskCreatePinnedToCore(
        WiFiTask,   /* Task function */
        "WiFiTask", /* Name of task */
        10000,      /* Stack size of task */
        this,       /* Parameter of the task */
        1,          /* Priority of the task */
        NULL,       /* Task handle to keep track of created task */
        0);         /* Core where the task should run */
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
            Serial.print("Connected to WiFi: ");
            Serial.print(WiFi.localIP());
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
    PIDgains PID = pid.getGains();

    File file = SPIFFS.open("/pid_values.txt", FILE_WRITE);
    if (!file)
    {
        Serial.println("There was an error opening the file for writing");
        return false;
    }

    if (motorsOff())
    {
        // Only write roll and yaw values, since pitch will mirror roll
        file.printf("P_GAIN_ROLL:%f\n", PID.p_gain_roll);
        file.printf("I_GAIN_ROLL:%f\n", PID.i_gain_roll);
        file.printf("D_GAIN_ROLL:%f\n", PID.d_gain_roll);

        file.printf("P_GAIN_YAW:%f\n", PID.p_gain_yaw);
        file.printf("I_GAIN_YAW:%f\n", PID.i_gain_yaw);
        file.printf("D_GAIN_YAW:%f\n", PID.d_gain_yaw);

        // //print to test 
        // Serial.println("PID values saved to SPIFFS:");
        // Serial.print("P Gain Roll: "); Serial.println(PID.p_gain_roll);
        // Serial.print("I Gain Roll: "); Serial.println(PID.i_gain_roll);
        // Serial.print("D Gain Roll: "); Serial.println(PID.d_gain_roll);
        // Serial.print("P Gain Yaw: "); Serial.println(PID.p_gain_yaw);
        // Serial.print("I Gain Yaw: "); Serial.println(PID.i_gain_yaw);
        // Serial.print("D Gain Yaw: "); Serial.println(PID.d_gain_yaw);

    }
    file.close();
    return true;
}

bool PID_Webserver::loadPIDValues()
{
    PIDgains PID = pid.getGains();    

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
            PID.p_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            PID.p_gain_pitch = PID.p_gain_roll;
        }
        else if (line.startsWith("I_GAIN_ROLL:"))
        {
            PID.i_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            PID.i_gain_pitch = PID.i_gain_roll;
        }
        else if (line.startsWith("D_GAIN_ROLL:"))
        {
            PID.d_gain_roll = line.substring(line.indexOf(':') + 1).toFloat();
            // Mirror the roll values to pitch
            PID.d_gain_pitch = PID.d_gain_roll;
        }
        else if (line.startsWith("P_GAIN_YAW:"))
        {
            PID.p_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("I_GAIN_YAW:"))
        {
            PID.i_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
        else if (line.startsWith("D_GAIN_YAW:"))
        {
            PID.d_gain_yaw = line.substring(line.indexOf(':') + 1).toFloat();
        }
    }

    file.close();
    pid.setGains(PID);
    return true;
}

String formatFloat(float value, unsigned int maxDecimals)
{
    char buffer[32]; // Make sure the buffer is large enough to hold the largest possible number
    // Create a format string for snprintf that specifies the maximum number of decimal places
    snprintf(buffer, sizeof(buffer), "%.*f", maxDecimals, value);

    // Trim trailing zeros and the decimal point if not needed
    char *end = buffer + strlen(buffer) - 1;
    while (end > buffer && *end == '0')
        --end;
    if (end > buffer && *end == '.')
        --end;
    *(end + 1) = '\0'; // Null-terminate the string

    return String(buffer);
}

void PID_Webserver::fillPIDJson(DynamicJsonDocument &doc)
{

    const PIDgains PID = pid.getGains();
    const PIDOut PID_out = pid.getOutput();

    // Roll PID parameters
    doc["pid_p_gain_roll"] = formatFloat(PID.p_gain_roll, 3);
    doc["pid_i_gain_roll"] = formatFloat(PID.i_gain_roll, 3);
    doc["pid_d_gain_roll"] = formatFloat(PID.d_gain_roll, 3);
    doc["pid_max_roll"] = formatFloat(PID_out.max_roll, 3);

    // Pitch PID parameters
    doc["pid_p_gain_pitch"] = formatFloat(PID.p_gain_pitch, 3);
    doc["pid_i_gain_pitch"] = formatFloat(PID.i_gain_pitch, 3);
    doc["pid_d_gain_pitch"] = formatFloat(PID.d_gain_pitch, 3);
    doc["pid_max_pitch"] = formatFloat(PID_out.max_pitch, 3);

    // Yaw PID parameters
    doc["pid_p_gain_yaw"] = formatFloat(PID.p_gain_yaw, 3);
    doc["pid_i_gain_yaw"] = formatFloat(PID.i_gain_yaw, 3);
    doc["pid_d_gain_yaw"] = formatFloat(PID.d_gain_yaw, 3);
    doc["pid_max_yaw"] = formatFloat(PID_out.max_yaw, 3);

}

String PID_Webserver::updatePIDFromRequest(AsyncWebServerRequest *request)
{
    // PIDgains PID = pid.getGains(); // Get current PID gains to update only the ones provided in the request
    //We should be setting the gains here not getting them 
    PIDgains PID; 

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
    updateParamFloat("pid_p_gain_roll", PID.p_gain_roll);
    updateParamFloat("pid_i_gain_roll", PID.i_gain_roll);
    updateParamFloat("pid_d_gain_roll", PID.d_gain_roll);

    // Assign roll PID values to pitch and yaw as well
    PID.p_gain_pitch = PID.p_gain_roll;
    PID.i_gain_pitch = PID.i_gain_roll;
    PID.d_gain_pitch = PID.d_gain_roll;

    // The following yaw was assigned to roll instead of yaw, 
    PID.p_gain_yaw = PID.p_gain_yaw; 
    PID.i_gain_yaw = PID.i_gain_yaw;
    PID.d_gain_yaw = PID.d_gain_yaw;

    // Update Yaw PID parameters
    updateParamFloat("pid_p_gain_yaw", PID.p_gain_yaw);
    updateParamFloat("pid_i_gain_yaw", PID.i_gain_yaw);
    updateParamFloat("pid_d_gain_yaw", PID.d_gain_yaw);

    if (response.isEmpty())
    {
        response = "No parameters updated.";
    }

    pid.setGains(PID); // Update the PID gains with the new values
    return response;
}

void PID_Webserver::handleSetPID(AsyncWebServerRequest *request)
{
    if (motorsOff())
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
    if (motorsOff())
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
