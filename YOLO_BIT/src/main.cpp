#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DHT20.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <string.h>
#include <time.h>
#include <LiquidCrystal_I2C.h>
#include <config.h>

LiquidCrystal_I2C lcd(0x21, 16, 2);

#define LED_PIN     32   
#define LED_COUNT   4    
#define BUTTON_PIN  35   
#define LightCensor 33
#define FAN_PIN     27

// #define WIFI_SSID     "binbin"
// #define WIFI_PASSWORD "0906640081"

FirebaseData firebaseData;
FirebaseJson json;
FirebaseConfig config;
FirebaseAuth auth;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setAllLEDs(uint32_t color, uint8_t brightness) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = (r * brightness) / 100;
    g = (g * brightness) / 100;
    b = (b * brightness) / 100;

    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

bool initializeSensorsAndOutputs(DHT20 &dht20) {
    Wire.begin(GPIO_NUM_21, GPIO_NUM_22);
    Wire.setClock(100000);
    if (!dht20.begin()) {
        Serial.println("DHT20 initialization failed!");
        return false;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);

    lcd.init();
    lcd.backlight();
    lcd.clear();

    return true;
}

bool checkConnections(bool &connectionsEstablished) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Disconnected, skipping all operations");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connecting to WiFi");
        lcd.setCursor(0, 1);
        lcd.print("Please wait...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return false;
    }

    if (firebaseData.httpCode() < 0) {
        Serial.print("Firebase connection error: ");
        Serial.println(firebaseData.errorReason());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connecting to Firebase");
        lcd.setCursor(0, 1);
        lcd.print("Please wait...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return false;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < 100) {
        Serial.println("Waiting for NTP synchronization...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Syncing NTP Time");
        lcd.setCursor(0, 1);
        lcd.print("Please wait...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return false;
    }

    if (!connectionsEstablished) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("All Connections OK");
        lcd.setCursor(0, 1);
        lcd.print("Starting...");
        Serial.println("All connections established successfully!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        connectionsEstablished = true;
    }

    return true;
}

void sendSensorData(DHT20 &dht20) {
    float lightIntensity = analogRead(LightCensor);
    Serial.print("Light intensity: ");
    Serial.println(lightIntensity);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (!dht20.read()) {
        Serial.println("DHT20 read failed, retrying...");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    Serial.printf("Temp: %.2f, Humidity: %.2f\n", temperature, humidity);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    time_t now;
    time(&now);
    Serial.print("Current timestamp: ");
    Serial.println((long)now);

    json.clear();
    json.set("light_intensity", lightIntensity);
    json.set("temperature", temperature);
    json.set("humidity", humidity);
    json.set("timestamp", (long)now);
    String uid = "/data/hLti6ttX5cQ04pFMnFAG";
    if (Firebase.RTDB.setJSON(&firebaseData, uid, &json)) {
        Serial.println("Sensor data sent to Firebase RTDB!");
    } else {
        Serial.print("RTDB Error: ");
        Serial.println(firebaseData.errorReason());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    json.clear();
    json.set("fields/sensorId/stringValue", "hLti6ttX5cQ04pFMnFAG"); 
    json.set("fields/light_intensity/doubleValue", lightIntensity);
    json.set("fields/temperature/doubleValue", temperature);
    json.set("fields/humidity/doubleValue", humidity);
    json.set("fields/timestamp/integerValue", (long)now);

    String documentPath = "user_sensor";
    if (Firebase.Firestore.createDocument(&firebaseData, PROJECT_ID, "", documentPath.c_str(), json.raw())) {
        Serial.println("Sensor data sent to Firestore!");
    } else {
        Serial.print("Firestore Error: ");
        Serial.println(firebaseData.errorReason());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature, 1);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(humidity, 1);
    lcd.print("%");
}

void controlDevices() {
    int retryCount = 0;
    bool fanSuccess = false;
    int fanState = 0; // Mặc định tắt
    while (retryCount < 3 && !fanSuccess) {
        if (Firebase.RTDB.getInt(&firebaseData, "/control/hLti6ttX5cQ04pFMnFAG/button_for_fan")) {
            if (firebaseData.dataType() == "int") {
                fanState = firebaseData.intData();
                Serial.print("Fan state: ");
                Serial.println(fanState);
                fanSuccess = true;
            } else {
                Serial.println("Error reading button_for_fan: Invalid data type");
            }
        } else {
            Serial.print("Error reading button_for_fan: ");
            Serial.println(firebaseData.errorReason());
        }
        if (!fanSuccess) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            retryCount++;
        }
    }

    // Điều chỉnh tốc độ quạt dựa trên fanState
    int fanSpeed = 0;
    switch (fanState) {
        case 3: // Mạnh
            fanSpeed = 255; // 100%
            break;
        case 2: // Mạnh vừa
            fanSpeed = 192; // 75%
            break;
        case 1: // Trung bình
            fanSpeed = 128; // 50%
            break;
        case 0: // Tắt
        default:
            fanSpeed = 0; // 0%
            break;
    }
    analogWrite(FAN_PIN, fanSpeed); // Sử dụng analogWrite để điều khiển quạt
    Serial.print("Fan speed (analogWrite): ");
    Serial.println(fanSpeed);

    retryCount = 0;
    bool ledSuccess = false;
    int ledState = 0; // Mặc định tắt
    while (retryCount < 3 && !ledSuccess) {
        if (Firebase.RTDB.getInt(&firebaseData, "/control/hLti6ttX5cQ04pFMnFAG/button_for_led")) {
            if (firebaseData.dataType() == "int") {
                ledState = firebaseData.intData();
                Serial.print("LED state: ");
                Serial.println(ledState);
                ledSuccess = true;
            } else {
                Serial.println("Error reading button_for_led: Invalid data type");
            }
        } else {
            Serial.print("Error reading button_for_led: ");
            Serial.println(firebaseData.errorReason());
        }
        if (!ledSuccess) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            retryCount++;
        }
    }

    retryCount = 0;
    bool brightnessSuccess = false;
    int brightness = 0; // Mặc định độ sáng là 0
    while (retryCount < 3 && !brightnessSuccess) {
        if (Firebase.RTDB.getInt(&firebaseData, "/control/hLti6ttX5cQ04pFMnFAG/candel_power_for_led")) {
            if (firebaseData.dataType() == "int") {
                brightness = firebaseData.intData();
                if (brightness < 0) brightness = 0;
                if (brightness > 100) brightness = 100;
                Serial.print("LED brightness: ");
                Serial.println(brightness);
                brightnessSuccess = true;
            } else {
                Serial.println("Error reading candel_power_for_led: Invalid data type");
            }
        } else {
            Serial.print("Error reading candel_power_for_led: ");
            Serial.println(firebaseData.errorReason());
        }
        if (!brightnessSuccess) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            retryCount++;
        }
    }

    // Điều chỉnh đèn LED dựa trên trạng thái và độ sáng
    uint32_t color = strip.Color(0, 0, 0); // Mặc định tắt
    switch (ledState) {
        case 4: // Cold - Xanh đậm
            color = strip.Color(0, 0, 139);
            break;
        case 3: // Cool - Trắng
            color = strip.Color(255, 255, 255);
            break;
        case 2: // Warm - Vàng
            color = strip.Color(255, 215, 0);
            break;
        case 1: // Hot - Cam
            color = strip.Color(255, 165, 0);
            break;
        case 0: // Tắt
        default:
            color = strip.Color(0, 0, 0);
            brightness = 0; // Đặt độ sáng về 0 khi tắt
            break;
    }
    setAllLEDs(color, brightness);
}

void communicate_with_firebase(void *pvParameters) {
    DHT20 dht20;
    bool connectionsEstablished = false;

    if (!initializeSensorsAndOutputs(dht20)) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    while (1) {
        if (!checkConnections(connectionsEstablished)) {
            continue;
        }

        static unsigned long lastSensorSend = 0;
        if (millis() - lastSensorSend >= 5000) {
            sendSensorData(dht20);
            lastSensorSend = millis();
        }

        static unsigned long lastControlCheck = 0;
        if (millis() - lastControlCheck >= 1000) {
            controlDevices();
            lastControlCheck = millis();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);
    strip.begin();
    strip.show();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    setAllLEDs(strip.Color(0, 0, 0), 0);

    // Không cần cấu hình PWM (ledc) nữa, analogWrite sẽ tự xử lý
    analogWrite(FAN_PIN, 0); // Tắt quạt ban đầu

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
    Serial.print("Connected to IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    configTime(0, 0, "pool.ntp.org");

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    while (timeinfo.tm_year < 100) {
        delay(500);
        Serial.print(".");
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    Serial.println("Time synchronized!");

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    xTaskCreate(communicate_with_firebase, "CommunicateWithFirebaseTask", 32768, NULL, 2, NULL);
}

void loop() {}