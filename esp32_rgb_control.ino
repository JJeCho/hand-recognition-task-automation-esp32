#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const int RED_PIN = 14;
const int GREEN_PIN = 12;
const int BLUE_PIN = 13;

const int LEDC_CHANNEL_RED = 0;
const int LEDC_CHANNEL_GREEN = 1;
const int LEDC_CHANNEL_BLUE = 2;

const int LEDC_TIMER_BITS = 8;
const int LEDC_BASE_FREQ = 5000;

// Replace with your network credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

WebServer server(80);

void setup() {
  Serial.begin(115200);

  // Initialize LED PWM channels
  ledcSetup(LEDC_CHANNEL_RED, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcSetup(LEDC_CHANNEL_GREEN, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcSetup(LEDC_CHANNEL_BLUE, LEDC_BASE_FREQ, LEDC_TIMER_BITS);

  // Attach pins to channels
  ledcAttachPin(RED_PIN, LEDC_CHANNEL_RED);
  ledcAttachPin(GREEN_PIN, LEDC_CHANNEL_GREEN);
  ledcAttachPin(BLUE_PIN, LEDC_CHANNEL_BLUE);

  setColor(0, 0, 0);
  Serial.println("RGB LED control ready...");

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up HTTP server routes
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);

  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    processCommand(cmd);
    server.send(200, "text/plain", "Command received: " + cmd);
  } else {
    server.send(200, "text/plain", "No command received");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void processCommand(String inputString) {
  inputString.trim();
  Serial.println("Received command: " + inputString);

  if (inputString == "CMD_OPEN_PALM") {
    Serial.println("Setting color: White");
    setColor(255, 255, 255);     // White
  } 
  else if (inputString == "CMD_CLOSED_FIST") {
    Serial.println("Setting color: Red");
    setColor(255, 0, 0);         // Red
  } 
  else if (inputString == "CMD_POINTING_INDEX") {
    Serial.println("Setting color: Green");
    setColor(0, 255, 0);         // Green
  } 
  else if (inputString == "CMD_PINCH") {
    Serial.println("Setting color: Blue");
    setColor(0, 0, 255);         // Blue
  } 
  else if (inputString == "CMD_CLICK") {
    Serial.println("Setting color: Yellow");
    setColor(255, 255, 0);       // Yellow
  } 
  else if (inputString == "CMD_SWIPE_LEFT") {
    Serial.println("Setting color: Cyan");
    setColor(0, 255, 255);       // Cyan
  } 
  else if (inputString == "CMD_SWIPE_RIGHT") {
    Serial.println("Setting color: Magenta");
    setColor(255, 0, 255);       // Magenta
  } 
  else if (inputString == "CMD_POINTING_MIDDLE") {
    Serial.println("Setting color: Orange");
    setColor(255, 120, 0);       // Vibrant Orange
  } 
  else if (inputString == "CMD_POINTING_RING") {
    Serial.println("Setting color: Purple");
    setColor(128, 0, 128);       // Purple
  } 
  else if (inputString == "CMD_POINTING_PINKY") {
    Serial.println("Setting color: Pink");
    setColor(255, 105, 180);     // Vibrant Pink (Hot Pink)
  } 
  else if (inputString == "CMD_HANG_LOOSE") {
    Serial.println("Setting color: Lime");
    setColor(191, 255, 0);       // Lime
  }
  else if (inputString == "CMD_CALL_ME") {
    Serial.println("Setting color: Sky Blue");
    setColor(135, 206, 235);     // Sky Blue
  }
  else if (inputString == "CMD_OK_SIGN") {
    Serial.println("Setting color: Brown");
    setColor(139, 69, 19);       // Brown
  }
  else if (inputString == "CMD_THUMBS_UP") {
    Serial.println("Setting color: Indigo");
    setColor(75, 0, 130);        // Indigo
  }
  else if (inputString == "CMD_PEACE_SIGN") {
    Serial.println("Setting color: Violet");
    setColor(238, 130, 238);     // Violet
  }
  else if (inputString == "CMD_THREE_FINGERS") {
    Serial.println("Setting color: Gold");
    setColor(255, 215, 0);       // Gold
  }
  else if (inputString == "CMD_ROCK_ON") {
    Serial.println("Setting color: Turquoise");
    setColor(64, 224, 208);      // Turquoise
  }
  else if (inputString == "CMD_SPIDERMAN") {
    Serial.println("Setting color: Navy");
    setColor(0, 0, 128);         // Navy
  }
  else {
    Serial.println("Unknown command, turning off LED...");
    setColor(0, 0, 0);           // Turn off LEDs
  }
}

void setColor(int red, int green, int blue) {
  // Invert colors if necessary (depends on your LED configuration)
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;

  ledcWrite(LEDC_CHANNEL_RED, red);
  ledcWrite(LEDC_CHANNEL_GREEN, green);
  ledcWrite(LEDC_CHANNEL_BLUE, blue);
}
