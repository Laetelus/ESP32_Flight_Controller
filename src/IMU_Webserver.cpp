  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <Wire.h>
  #include <Arduino_JSON.h>
  #include "SPIFFS.h"
  #include "Flight_Controller.h"
  // Replace with your network credentials
  const char* ssid = "Untrusted_Network";
  const char* password = "rapidcream878";

  // Create AsyncWebServer object on port 80
  AsyncWebServer server(80);

  // Create an Event Source on /events
  AsyncEventSource events("/events");

  // Json Variable to Hold Sensor Readings
  JSONVar readings;

  void Flight_Controller::initSPIFFS() {
    if (!SPIFFS.begin()) {
      Serial.println("An error has occurred while mounting SPIFFS");
    }
      Serial.println("SPIFFS mounted successfully");
  }

  // Initialize WiFi
  void Flight_Controller::initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("");
    Serial.println(WiFi.localIP());
  }

 void Flight_Controller::Handle_Server(){
  
    // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void Flight_Controller::Send_Event(){ 
  readings["gyro_roll"] = String(angle_roll);
  readings["gyro_pitch"] = String(angle_pitch);
  readings["gyro_yaw"] = String(gyro_yaw_input);

  readings["acc_x"] = String(ax_mps2);
  readings["acc_y"] = String(ay_mps2);
  readings["acc_z"] = String(az_mps2);

  String jsonString = JSON.stringify(readings);
  // Send Events to the Web Server with the Sensor Readings
    events.send(jsonString.c_str(),"imu_readings",millis());
}



