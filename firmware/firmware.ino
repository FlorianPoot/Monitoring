// Copyright (c) 2021 Florian Poot
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <avr/wdt.h>
#include <Controllino.h>
#include <ArduinoJson.h>   // Version 6.18.5
#include <PubSubClient.h>  // Version 2.8.0
#include <Ethernet.h>
#include <Ticker.h>        // Version 4.4.0

#define PRODUCTION_LINE 1
const byte mac[] = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED};
IPAddress ip(172, 16, 0, 100);
IPAddress server(192, 168, 0, 131);

#define BUTTON_1  CONTROLLINO_AI0
#define BUTTON_2  CONTROLLINO_AI1
#define BUTTON_3  CONTROLLINO_AI2
#define BUTTON_4  CONTROLLINO_AI3
#define BUTTON_5  CONTROLLINO_AI4
#define BUTTON_6  CONTROLLINO_AI5

#define GREEN_LED CONTROLLINO_DO0
#define RED_LED   CONTROLLINO_DO1

#define SENSOR    CONTROLLINO_DI0


#define NORMAL_MODE  0
#define FAULT_MODE   1

// Prototypes
void fault(bool faulty=true);
bool send_data(char data_name[], uint16_t data);
void panel_buttons();
void error_led();

const uint8_t buttons[] = {BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, BUTTON_5, BUTTON_6};
uint8_t current_mode = NORMAL_MODE;

Ticker fault_timer(fault, 10000, 1);              // If no count in 10s, raise fault
Ticker error_led_timer(error_led, 1000);          // Blink error led at 0.5Hz
Ticker error_led_blink_timer(error_led, 250, 4);  // Blink 2 times at 2Hz

EthernetClient ethClient;
PubSubClient client(ethClient);


void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reconnect() {
    // Disable watchdog to prevent reset
    wdt_disable();
    
    // Enable error led
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    // Set fault to false, reconnection is a special type of error and not considered as fault
    fault(false);
    
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("controllinoClient1")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }

    // Normal mode
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    
    wdt_enable(WDTO_250MS);
}

void setup() {
    // Enable serial for debugging
    Serial.begin(9600);
    Controllino_RTC_init();
    // Controllino_SetTimeDateStrings(__DATE__, __TIME__);  // Set compilation time to the RTC chip
    
    // Panel LEDs
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    
    Serial.println("Controllino initialized");
    // Set error led when starting
    digitalWrite(RED_LED, HIGH);
     
    // Set client settings
    client.setServer(server, 1883);
    client.setCallback(callback);
    Serial.print("Start ethernet... ");
    Ethernet.begin(mac, ip);
    // Allow the hardware to sort itself out
    delay(1500);
    Serial.print("ok, ip=");
    Serial.println(Ethernet.localIP());
    
    Serial.println("Initialize inputs and outputs");
    // Initialize inputs and outputs
    for (uint8_t i = 0; i < 6; i++) {
        pinMode(buttons[i], INPUT_PULLUP);
    }
    pinMode(SENSOR, INPUT);
    
    // Enable watchdog (250ms)
    wdt_enable(WDTO_250MS);
}

void loop() {
    static uint16_t counting_value = 0;
    uint32_t start_time = millis();

    // If connection is lost, reconnect
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    static bool sensor_state = false;
    bool sensor_value = digitalRead(SENSOR);
    if (sensor_value == true && sensor_value != sensor_state) {
        Serial.println("count");
        counting_value++;

        if (current_mode == FAULT_MODE) {
            // Unknown reasons
            send_data("error", 7);
        }

        // Reset timer
        fault_timer.stop();
        fault_timer.start();
        fault(false);
    }
    sensor_state = sensor_value;

    // Send data to the server every minute
    static int8_t last_minute_value = -1;
    int8_t current_minute = Controllino_GetMinute();
    if (current_minute != last_minute_value) {
        if (send_data("count", counting_value))
            counting_value = 0;  // Reset counting value when data is sent
        last_minute_value = current_minute;
    }

    // Update timers
    fault_timer.update();
    error_led_timer.update();
    error_led_blink_timer.update();
    
    // Check if any buttons is press
    panel_buttons();
    
    // Only needed when using DHCP
    // Ethernet.maintain();
    
    // Reset watchdog
    wdt_reset();

    Serial.print("execution time=");
    Serial.println(millis() - start_time);
}

void panel_buttons() {
    static bool buttons_state[] = {false, false, false, false, false, false};

    for (uint8_t i = 0; i < 6; i++) {
        bool value = digitalRead(buttons[i]);

        if (value && value != buttons_state[i]) {
            Serial.println("ok");
            switch (current_mode) {
                case NORMAL_MODE:
                    // Not allowed
                    if (error_led_blink_timer.state() == STOPPED)
                        error_led_blink_timer.start();
                    break;
                case FAULT_MODE:
                    // Send to server error number (1-6)
                    send_data("error", i + 1);
                    fault(false);
                    break;
            }
        }
        buttons_state[i] = value;
    }
}

void fault(bool faulty=true) {
    if (!faulty) {
        current_mode = NORMAL_MODE;

        if (error_led_timer.state() == RUNNING) {
            error_led_timer.stop();
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);
        }
    } else {
        current_mode = FAULT_MODE;

        if (error_led_timer.state() == STOPPED) {
            error_led_timer.start();
            digitalWrite(GREEN_LED, LOW);

            // Send error
            send_data("error", 0);  // 0 means line stopped for unknown reason
        }
    }
}

void get_time(char time_buffer[]) {
    uint8_t day, weekday, month, year, hour, minute, second;
    Controllino_ReadTimeDate(&day, &weekday, &month, &year, &hour, &minute, &second);
    sprintf(time_buffer, "20%02d-%02d-%02dT%02d:%02d:%02d+00:00", year, month, day, hour, minute, second);  // ISO8601
}

bool send_data(char data_name[], uint16_t data) {
    StaticJsonDocument<256> JSONdata;

    char time_buffer[40];
    uint8_t day, weekday, month, year, hour, minute, second;
    Controllino_ReadTimeDate(&day, &weekday, &month, &year, &hour, &minute, &second);
    sprintf(time_buffer, "20%02d-%02d-%02dT%02d:%02d:%02d+00:00", year, month, day, hour, minute, second);  // ISO8601
    
    JSONdata["time"] = time_buffer;
    JSONdata["sensor"] = PRODUCTION_LINE;
    JSONdata[data_name] = data;

    char JSONdata_buffer[256];
    serializeJson(JSONdata, JSONdata_buffer);
    Serial.println("Sending data to MQTT topic..");
    Serial.println(JSONdata_buffer);
    
    // Send data over MQTT
    if (client.publish("monitoring", JSONdata_buffer)) {
        // If succeed
        Serial.println("Data sent!");
        return true;
    } else {
        Serial.println("Failed to send data");
        return false;
    }
}

void error_led() {
    // Invert led for blinking.
    digitalWrite(RED_LED, !digitalRead(RED_LED));
}
