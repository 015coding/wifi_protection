#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include <MFRC522.h>
#include <vector>
#include <string>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <esp_wifi.h>
#define TOPIC_STATUS      TOPIC_PREFIX "/board_2/status"
#define TOPIC_MODE        TOPIC_PREFIX "/board_2/mode"
#define TOPIC_FAKE_SSID   TOPIC_PREFIX "/board_2/ssid_open"
#define TOPIC_SET_MODE    TOPIC_PREFIX "/board_2/mode_sub"


#define SS_PIN 5        // RC522 SDA (Chip Select)
#define RST_PIN 22      // RC522 Reset
#define BUZZER_PIN 8  // Buzzer GPIO (Changed from GPIO 36)

// Updated SPI Pins (Safe for ESP32-S3)
#define SCK_PIN 18      // SPI Clock
#define MOSI_PIN 11     // SPI MOSI (Changed from GPIO 21)
#define MISO_PIN 13     // SPI MISO (Changed from GPIO 19)

//screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define VRY_PIN 6  // Use a safe ADC pin for joystick Y-axis
#define SW_PIN  7  // Use a safe digital pin for joystick button

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WiFiClient wifiClient;
PubSubClient mqtt(MQTT_BROKER, 1883, wifiClient);
uint32_t last_publish;
void Switch_case(int mode) {
  switchMode(mode);  // Redirect to switchMode
}

enum Mode { OFF, ADD_NEW_MAC, FILTERED_DETECT };
Mode currentMode = OFF;

void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  printf("WiFi MAC address is %s\n", WiFi.macAddress().c_str());
  printf("Connecting to WiFi %s.\n", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    printf(".");
    fflush(stdout);
    delay(500);
  }
  printf("\nWiFi connected.\n");
}


void connect_mqtt() {
  printf("Connecting to MQTT broker at %s.\n", MQTT_BROKER);
  if (!mqtt.connect("", MQTT_USER, MQTT_PASS)) {
    printf("Failed to connect to MQTT broker.\n");
    for (;;) {} // wait here forever
  }
  mqtt.setCallback(mqtt_callback);
  mqtt.subscribe(TOPIC_SET_MODE);
  mqtt.subscribe(TOPIC_FAKE_SSID);/////////////////////fake ssid 
  printf("MQTT broker connected.\n");
  Serial.println(TOPIC_FAKE_SSID);
  
}

///////////////////////////    Wifi config ///////////////////////
String ssid = "IOT-KU";  // Now it can be changed dynamically
const int channel = 6;
const int maxConnections = 4;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_SET_MODE) == 0) {
    payload[length] = 0; // null-terminate the payload to treat it as a string
    int value = atoi((char*)payload); 
    if (value == 1) {
      Switch_case(ADD_NEW_MAC);
    }
    else if (value == 2) {
      Switch_case(FILTERED_DETECT);
      Serial.println("connected mqtt");
    }
    else {
      printf("Invalid payload received.\n");
    }
  } else if (strcmp(topic, TOPIC_FAKE_SSID) == 0){
    payload[length] = '\0';
    ssid = String((char*)payload);
    Serial.printf("Fake SSID updated: %s\n", ssid.c_str());
  }
}


////// mode ////

//////MAc Address Storage///
std::vector<String> trustedMACs;
std::vector<String> knownDevices;

/// mac to strings ///
String macToString(const uint8_t* mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

/// add trusted mac ///
void addTrustedMAC(String mac) {
  if (std::find(trustedMACs.begin(), trustedMACs.end(), mac) == trustedMACs.end()) {
    trustedMACs.push_back(mac);
    Serial.printf(" MAC added to whitelist: %s\n", mac.c_str());
  } else {
    Serial.printf("ℹ MAC already whitelisted: %s\n", mac.c_str());
  }
}

/// new connect ///

void onConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_event_ap_staconnected_t* connected = (wifi_event_ap_staconnected_t*)&info;
  String mac = macToString(connected->mac);  //  Converts int to const char*
  

  if (std::find(knownDevices.begin(), knownDevices.end(), mac) == knownDevices.end()) {
    Serial.printf("\n New device connected: %s\n", mac.c_str());
    knownDevices.push_back(mac);


    if (currentMode == ADD_NEW_MAC) {
      addTrustedMAC(mac);
    }

    if (currentMode == FILTERED_DETECT && 
        std::find(trustedMACs.begin(), trustedMACs.end(), mac) == trustedMACs.end()) {
      Serial.println(" Unauthorized MAC detected! Disconnecting...");
      esp_wifi_disconnect();
      
    } else {
      Serial.println(" Connection allowed.");
      
    }
  }
}

// Disconnection //
void onDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifi_event_ap_stadisconnected_t* disconnected = (wifi_event_ap_stadisconnected_t*)&info;
  String mac = macToString(disconnected->mac);
  Serial.printf(" Device disconnected: %s\n", mac.c_str());

  auto it = std::find(knownDevices.begin(), knownDevices.end(), mac);
  if (it != knownDevices.end()) {
    knownDevices.erase(it);
    Serial.println(" MAC removed from known list.");
  }
}


// start fake ap //
void startFakeAP() {
  display.clearDisplay();
  updateDisplay("startFakeAP", 1);
  WiFi.softAP(ssid, "", channel, 0, maxConnections);
  Serial.printf("\n Fake AP '%s' started on channel %d.\n", ssid, channel);
  WiFi.onEvent(onConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(onDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  updateDisplay((String(ssid) + " channel " + String(channel)).c_str(), 4);
  
}

// Stop Fake AP


void resetWiFi() {
    Serial.println("\n [DEBUG] Resetting WiFi Stack...");
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_AP);
}


void switchMode(int mode) {
      // Always stop AP first
    currentMode = static_cast<Mode>(mode);
    knownDevices.clear();

    switch (currentMode) {
        case OFF:
            display.clearDisplay();
            
            Serial.println("\n Mode: OFF (AP disabled, Connecting to MQTT)");
            WiFi.softAPdisconnect(true);
            delay(500);

            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASS);

            Serial.print("Connecting to WiFi");
            while (WiFi.status() != WL_CONNECTED) {
                Serial.print(".");
                delay(500);
            }
            Serial.println("\n WiFi Connected!");

            connect_mqtt();
            
            updateDisplay("Mode: OFF", 2);
            updateDisplay("MQTT Connected", 4);
            mqtt.publish(TOPIC_MODE, "OFF MODE");
            mqtt.publish(TOPIC_STATUS,"ONLINE");
            break;

        case ADD_NEW_MAC:
            display.clearDisplay();
            Serial.println("\n Mode: ADD NEW MAC (Switching from STA to AP)");
            mqtt.publish(TOPIC_STATUS,"OFFLINE");
            WiFi.disconnect(true);
            WiFi.softAPdisconnect(true);
            resetWiFi();  //  Add this to fully reset the WiFi stack
            
            WiFi.mode(WIFI_AP);
            startFakeAP();
            
            updateDisplay("Mode: ADD NEW MAC", 3);
            break;

        case FILTERED_DETECT:
            display.clearDisplay();
            mqtt.publish(TOPIC_STATUS,"OFFLINE");
            Serial.println("\n Mode: FILTERED DETECT (Only trusted MACs allowed)");
            WiFi.disconnect(true);
            WiFi.softAPdisconnect(true);
            resetWiFi();  //  Reset before switching to AP Mode
            WiFi.mode(WIFI_AP);
            
            
            updateDisplay("Mode: FILTERED DETECT", 1);
            delay(3000);
            startFakeAP();
            break;

        default:
            Serial.println("X Invalid Mode!");
            break;
    }
}




////////////////////////////////rfid//////////////////////////////
MFRC522 rfid(SS_PIN, RST_PIN);
byte adminUID[] = {0x13, 0xAE, 0x23, 0x28};  

bool isAdminCard(byte *uid, byte size) {
  if (size != sizeof(adminUID)) {
    return false;
  }
  for (byte i = 0; i < size; i++) {
    if (uid[i] != adminUID[i]) {
      return false;
    }
  }
  return true;
}

void activateBuzzer(bool isAdmin) {
  int beepDuration = isAdmin ? 300 : 1000;
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(beepDuration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(300);
  }
}


void displayCenteredText(const char* text, int line) {
    // display.clearDisplay();

    int16_t x1, y1;
    uint16_t width, height;
    display.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);
    int x = (SCREEN_WIDTH - width) / 2;
    int y = (line * 10);
    display.setCursor(x, y);
    display.println(text);
    display.display();
}

void updateDisplay(const char* text,int num) {
    // display.clearDisplay();
  // if (text != "clear") {

    displayCenteredText(text, num);
    Serial.println(text);
    display.display();
  // } else {
  //   display.clearDisplay();
  //   Serial.println("DEBUG display");
  // }
}

void updateDisplayLeft(const char* text, int line) {

    int x = 0;  // Start from the left edge
    int y = line * 10;  // Line spacing (adjust as needed)

    
    display.setCursor(x, y);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println(text);
    display.display();
    Serial.println(text);

}

////// update //////
void updateMenu(int selectedMode) {
    updateDisplayLeft("Select Mode:", 1);
    updateDisplayLeft(selectedMode == 0 ? "> 0 (OFF)" : "  0 (OFF)", 2);
    updateDisplayLeft(selectedMode == 1 ? "> 1 (Add MAC)" : "  1 (Add MAC)", 3);
    updateDisplayLeft(selectedMode == 2 ? "> 2 (Filtered Detect)" : "  2 (Filtered Detect)", 4);
}

//joys
String getJoystickDirection(int yValue) {
    if (yValue > 3500) { 
        return "Up";
    } else if (yValue < 600) { 
        return "Down";
    } else {
        return "Center";
    }
}

String getButton(int buttonState) {
    return buttonState == 1 ? "notpress" : "press";
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);  // Ensure Buzzer works
  Wire.begin(15,16);
  Serial.begin(115200);

  //connecting wifi
  connect_wifi();
  connect_mqtt();

    // Setup SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();  // Initialize MFRC522

  /// joys
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(VRY_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  Serial.println("\n Enter mode (0=OFF, 1=ADD_NEW_MAC, 2=FILTERED_DETECT):");
  if (!display.begin(0x3C, true)) {  // 0x3C is the common I2C address for SH1106
      Serial.println(F("SH1106 not found!"));
      while (1); // Stop execution if OLED is not found
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);  
  displayCenteredText("Setup started",3);
  delay(2000);
  display.clearDisplay();
  displayCenteredText("...Running...", 3);
  display.display();
  delay(1000);

  // display.display();
  mqtt.publish(TOPIC_MODE,"OFF");
  mqtt.publish(TOPIC_STATUS,"ONLINE");





  display.clearDisplay();
  last_publish = 0;

  

}


void loop() {
    mqtt.loop();
  
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        delay(100);
        return;
    }

    if (isAdminCard(rfid.uid.uidByte, rfid.uid.size)) {
        Serial.println("[/] Admin detected! Showing mode selection...");
        mqtt.publish(TOPIC_STATUS, "Admin detected!");
        activateBuzzer(true);
        display.clearDisplay();
        updateDisplayLeft("Admin detected!", 1);
        
        display.display();
        delay(1000);
        display.clearDisplay();

        // Joystick-based mode selection
        updateDisplayLeft("Select Mode:", 1);
        updateDisplayLeft("> 0 (OFF)", 2);
        updateDisplayLeft("  1 (Add MAC)", 3);
        updateDisplayLeft("  2 (Filtered Detect)", 4);
        display.clearDisplay();

        String lastDirection = "Center";
        String lastButtonState = "notpress";
        int selectedMode = 0;

        while (true) {  
          display.clearDisplay();
            int yValue = analogRead(VRY_PIN);
            String currentDirection = getJoystickDirection(yValue);
            int buttonState = digitalRead(SW_PIN);
            String currentButtonState = getButton(buttonState);

            if (currentDirection == "Up" && lastDirection != "Up") {
                selectedMode = (selectedMode - 1 + 3) % 3;
                updateMenu(selectedMode);
            } else if (currentDirection == "Down" && lastDirection != "Down") {
                selectedMode = (selectedMode + 1) % 3;
                updateMenu(selectedMode);
            }

            if (currentButtonState == "press" && lastButtonState != "press") {
                Serial.printf("\n🔄 Changing mode to %d...\n", selectedMode);
                digitalWrite(BUZZER_PIN, HIGH);
                delay(100);
                digitalWrite(BUZZER_PIN, LOW);
                //
                switchMode(selectedMode);
                break;  
            }

            lastDirection = currentDirection;
            lastButtonState = currentButtonState;
            delay(100);
        }

    } else {
        Serial.println("[X] Unknown card! Activating alert...");
        mqtt.publish(TOPIC_STATUS, "ALERT!");

        // Save the previous screen (current mode)
        bool isFilteredDetect = (currentMode == FILTERED_DETECT);
        display.clearDisplay();
        // Show "Unknown Card" for 5 seconds
        updateDisplayLeft("Unknown card!", 3);
        activateBuzzer(false);
        delay(5000);

        // Restore previous screen
        // display.clearDisplay();

        if (isFilteredDetect) {
            // Restore Fake AP screen exactly like `startFakeAP()`
            display.clearDisplay();
            updateDisplay("startFakeAP", 2);
            updateDisplay((String(ssid) + " channel " + String(channel)).c_str(), 4);
        } else {
            display.clearDisplay();
            // Restore normal mode text
            String previousDisplay = (currentMode == OFF) ? "Mode: OFF" : "Mode: ADD MAC";
            updateDisplay(previousDisplay.c_str(), 2);
        }
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

}



