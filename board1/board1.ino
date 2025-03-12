#include "WiFi.h"
#include "esp_wifi.h"
//#include <esp_now.h>
#include <map>
#include <set>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <config.h>
#include <PubSubClient.h>
#include <vector>
#include <Wire.h>

#define TOPIC_DETECT  TOPIC_PREFIX "/detected_1"
#define TOPIC_STATUS  TOPIC_PREFIX "/status_1"
#define TOPIC_MODE    TOPIC_PREFIX "/board_1/switch_mode"
#define TOPIC_SSID    TOPIC_PREFIX "/board_1/ssid"
#define TOPIC_BSSID   TOPIC_PREFIX "/board_1/bssid"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define PIN_LED 15  
#define BUZZER_PIN 40


std::map<String, int> attackCount;    // Store attack counts per BSSID
std::map<String, String> bssidToSsid; // Store BSSID to SSID mapping
std::set<String> detectedAttackers;   // Store detected attackers for single alert

uint8_t receiverAddress[] = {0x68,0xb6,0xb3,0x38,0x02,0xa4};  // Replace with the receiver MAC

typedef struct Message {
  char ssid[32];
} Message;

struct DeauthAttack {
    String bssid;
    String ssid;
};

std::vector<DeauthAttack> mqttQueue; // Queue to store detected attacks

Message message;
volatile bool switchToMQTT = false;

bool deauthDetected = false; 

enum Mode { sniff_mode, connect_mqttx };
Mode currentMode = connect_mqttx;
void Switch_case(int mode);

////// mqttx connected ///////
WiFiClient wifiClient;
PubSubClient mqtt(MQTT_BROKER, 1883, wifiClient);
uint32_t last_publish;


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
  mqtt.subscribe(TOPIC_MODE);
  // mqtt.subscribe(TOPIC_LED_RED);
  printf("MQTT broker connected.\n");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_MODE) == 0) {
    payload[length] = 0; // null-terminate the payload to treat it as a string
    int value = atoi((char*)payload); 
    if (value == 1) {
      Switch_case(sniff_mode);
      Serial.println("sniff_mode opened");
    }
    else if (value == 0) {
      Switch_case(connect_mqttx);
      Serial.println("connected mqtt");
    }
    else {
      printf("Invalid payload received.\n");
    }
  }
}


void switchToMQTTMode() {
    Serial.println("🔄 Switching to MQTT mode...");

    // 1️⃣ Stop Sniffer Mode Safely
    esp_wifi_set_promiscuous(false);  
    Serial.println("🚫 Promiscuous Mode Disabled");

    // 2️⃣ Fully Reset WiFi Stack to Avoid Deadlocks
    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);   // Turn WiFi off completely
    delay(1000);           // Give it time to reset

    // 3️⃣ Switch to WiFi Station Mode
    WiFi.mode(WIFI_STA);
    Serial.println("🌐 Switched to WiFi Station Mode");

    // 4️⃣ Connect to WiFi
    connect_wifi();

    // 5️⃣ Connect to MQTT
    connect_mqtt();

    // 6️⃣ Check MQTT Connection Before Publishing
    if (mqtt.connected()) {
        Serial.println("✅ MQTT Connected! Sending alert...");
        mqtt.publish(TOPIC_STATUS, "Deauth Attack Detected");
        Serial.println("📡 Alert Sent to MQTT Broker");
        DeauthAttack attack = mqttQueue.back();
        mqttQueue.pop_back();
        mqtt.publish(TOPIC_BSSID, attack.bssid.c_str());
        //mqtt.publish(TOPIC_SSID, attack.ssid.c_str());
        mqtt.publish(TOPIC_SSID, attack.ssid.isEmpty() ? "Unknown" : attack.ssid.c_str());

    } else {
        Serial.println("❌ MQTT Connection Failed. Could not publish.");
    }
}




// Function to display centered text on OLED
void displayCenteredText(const char* text, int line) {
    int16_t x1, y1;
    uint16_t width, height;
    display.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);
    int x = (SCREEN_WIDTH - width) / 2;
    int y = (line * 10);
    display.setCursor(x, y);
    display.println(text);
}

// Function to format MAC address
String formatMAC(const uint8_t* mac) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}



void sendMQTTAlert(const String& bssid, const String& ssid) {
    if (!mqtt.connected()) {
        Serial.println("❌ MQTT Not Connected. Cannot send alert.");
        return;
    }

    Serial.println("\n📡 Sending Deauth Attack Alert via MQTT...");
    
    // Publish BSSID
    mqtt.publish(TOPIC_BSSID, bssid.c_str());
    
    // Publish SSID (if empty, send "Unknown")
    mqtt.publish(TOPIC_SSID, ssid.isEmpty() ? "Unknown" : ssid.c_str());

    // Publish generic detection message
    mqtt.publish(TOPIC_DETECT, "Deauth Attack Detected");

    Serial.println("✅ MQTT Alert Sent!");
}


void extractSSID(const uint8_t* data, int len, const uint8_t* bssid) {
    int pos = 36;  // Start after MAC header

    while (pos < len) {
        uint8_t tagNumber = data[pos];
        uint8_t tagLength = data[pos + 1];

        if (tagNumber == 0x00) { // SSID Parameter Set
            if (tagLength > 0 && tagLength <= 32) {
                char ssid[33];
                memcpy(ssid, &data[pos + 2], tagLength);
                ssid[tagLength] = '\0';

                String bssidStr = formatMAC(bssid);
                bssidToSsid[bssidStr] = String(ssid);

                //Serial.printf("✅ Extracted SSID: %s for BSSID: %s\n", ssid, bssidStr.c_str());
            } else {
                //Serial.println("⚠️ Hidden SSID detected, waiting for probe response...");
            }
            break;
        }
        pos += tagLength + 2;  // Move to next tag
    }
}



// Function to update OLED display with attacker list
void updateDisplay(const String& bssid, const String& ssid) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    displayCenteredText("Deauth Attack!", 1);
    displayCenteredText(("BSSID:" + bssid).c_str(), 2);
    displayCenteredText(("SSID: " + (ssid.isEmpty() ? "Unknown" : ssid)).c_str(), 4);
    display.display();
}


void snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (!buf || type != WIFI_PKT_MGMT) return;

    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    uint8_t* frame = pkt->payload;

    if (pkt->rx_ctrl.sig_len < 28) return;

    uint16_t frameControl = (frame[0] | (frame[1] << 8));
    uint8_t frameType = (frameControl & 0x0C) >> 2;
    uint8_t frameSubType = (frameControl & 0xF0) >> 4;

    // Capture BSSID to SSID from Beacon/Probe Response
    if (frameType == 0x00 && (frameSubType == 0x08 || frameSubType == 0x05)) {
        extractSSID(frame, pkt->rx_ctrl.sig_len, &frame[16]);
    }

    // Detect Deauthentication frames (Subtype 0x0C)
    if (frameType == 0x00 && frameSubType == 0x0C) {
        String bssidStr = formatMAC(&frame[10]);

        // Only alert and update if a new attacker is detected
        if (detectedAttackers.find(bssidStr) == detectedAttackers.end()) {
            detectedAttackers.insert(bssidStr);
            String ssid = bssidToSsid[bssidStr];

            Serial.println("\n⚠️ New Deauth Attack Detected!");
            Serial.println("🚨 BSSID: " + bssidStr);
            Serial.println("🏷️ SSID: " + (ssid.isEmpty() ? "Unknown" : ssid));
            updateDisplay(bssidStr, ssid);
            
            mqttQueue.push_back({bssidStr, ssid});
            switchToMQTT = true;
            allert_message();
            
        }
    }
}

void allert_message() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(15, HIGH);  // Turn LED on
    delay(100);                   // Wait for 500ms
    digitalWrite(15, LOW);   // Turn LED off
    delay(100); 

    tone(BUZZER_PIN, 700);
    delay(100);
    noTone(BUZZER_PIN);
    delay(300);

  }
  
}

void resetWiFi() {
    Serial.println("\n🔄 [DEBUG] Resetting WiFi Stack...");
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_AP);
}

void Switch_case(int mode) {
    currentMode = static_cast<Mode>(mode);
    // knownDevices.clear();

    switch (currentMode) {
      case sniff_mode:
        //WiFi.disconnect(true);
        //WiFi.softAPdisconnect(true);
        delay(500);
        resetWiFi();
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(snifferCallback);
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        display.clearDisplay();
        displayCenteredText("Detector Enable", 3);
        display.display();
        Serial.println("🚀 ESP32 Deauth Detector with ESP-NOW Started...");
        break;
      case connect_mqttx:
        //WiFi.disconnect(true);
        //WiFi.softAPdisconnect(true);
        esp_wifi_set_promiscuous(false);
        delay(500);
        connect_wifi();
        connect_mqtt();
        display.clearDisplay();
        displayCenteredText("...connected...", 3);
        display.display();
        mqtt.publish(TOPIC_STATUS, "board 1 online");
        break;
    }
}


void setup() {
    Wire.begin(48, 47);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    pinMode(PIN_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("⚠️ OLED initialization failed.");
        while (1);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    displayCenteredText("Detector", 2);
    displayCenteredText("Running...", 3);
    display.display();
    Switch_case(connect_mqttx);




void loop() {
    static int channel = 1;
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    channel = (channel % 13) + 1;
    
    mqtt.loop();

    
    
    // ✅ Handle Mode Switching in Loop (outside interrupt)
    if (switchToMQTT) {
        switchToMQTT = false;  // Reset flag
        switchToMQTTMode();    // Call mode switch function
    }

    delay(200);

}

