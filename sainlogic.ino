#include <Arduino.h>
#include <WiFi.h>

#include "ESP32TimerInterrupt.h"
#include <PubSubClient.h>

#include "ring_buffer.h"
#include "ppm_tracker.h"
#include "data_decode.h"
#include "secrets.h"

// Define DEBUG_SAMPLER to log data packets to serial
// Instead of normal operation


// D1 GPIO5
// Connects to pin that goes low during receive
#define PIN_IN1 5
// D2 GPIO4
// Connects to pin that outputs received data
#define PIN_IN2 4

// Target time between samples
// Must be long enough that MCU can keep up with
// Expected samples/symbol = is SLEEP_US/46.3uS
#define SAMPLE_PERIOD_US 40

// Parameters for synching data
#define MIN_SAMPLES 4
#define MAX_SAMPLES 16

// MQTT Broker to connect to
const char* mqtt_server = "192.168.1.157";

WiFiClient wifi_client;
PubSubClient client(wifi_client);

// Init ESP32 timer
ESP32Timer ITimer(0);

BinaryPpmTracker tracker(MIN_SAMPLES, MAX_SAMPLES);

// setup WiFi based on ssid and password
// defined in gitignored secrets.h
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Don't listen for any MQTT topics
void callback(char* topic, byte* payload, unsigned int length) {
}

// Reconnect to MQTT Broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...1");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// Setup pins, serial, timer ISR, Wifi, and MQTT
void setup() {
  pinMode(PIN_IN1, INPUT);
  pinMode(PIN_IN2, INPUT);
  set_sample_pin(PIN_IN2);
  Serial.begin(115200);

  setup_wifi();
  Serial.println("Wi-Fi done");
  client.setServer(mqtt_server, 1883);
  Serial.println("MQTT server done");
  client.setCallback(callback);
  Serial.println("callback done");
  //client.setSocketTimeout(0xFFFF);
  client.setKeepAlive(0xFFFF);
  Serial.println("keep alive done");

  if (ITimer.attachInterruptInterval(SAMPLE_PERIOD_US, sample_input)) {
    Serial.println("if1");
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
    Serial.println("if1");
  } else {
    Serial.println("else");
    Serial.println("Can't set ITimer. Select another freq. or interval");
    Serial.println("else");
  }
}

#ifdef DEBUG_SAMPLER
/**
 * When debugging, log bit sequence following PIN_IN1 going low
 * DEBUG_SAMPLER causes sampler to capture a full buffers worth
 * after reset_sampler call. Last bit is not valid.
 */
void debug_loop() {
  static bool sent = true;
  if (sent) {
    if (!digitalRead(PIN_IN1)) {
      reset_sampler();
      sent = false;
    }
    return;
  }
  size_t buffered_len = num_samples();
  if (buffered_len == SAMPLE_LEN - 1) {
    sent = true;
    for (int i = 0; i < SAMPLE_LEN / 8; i++) {
      Serial.printf("%02X", get_sample_buffer()[i]);
    }
    Serial.print("\n");
    reset_sampler();
  }
}
#endif

void decode_and_publish(const uint8_t* msg) {
  if (check_crc(tracker.get_msg())) {
    String json_data = "{";
    json_data += String("\"temp_c\": ") + String(get_temperature(msg)) + ", ";
    json_data += String("\"humidity_%\": ") + String(get_humidity(msg)) + ", ";
    json_data += String("\"wind_dir_deg\": ") + String(get_direction(msg)) + ", ";
    json_data += String("\"avr_wind_m/s\": ") + String(get_avr_wind_speed(msg)) + ", ";
    json_data += String("\"gust_wind_m/s\": ") + String(get_gust_wind_speed(msg)) + ", ";
    json_data += String("\"rain_mm\": ") + String(get_rain(msg));
    json_data += "}";
    Serial.print(json_data);
    Serial.print('\n');
    client.publish("weather_decoded", json_data.c_str(), json_data.length());
  }
  Serial.print("CRC check failed\n");
}

void loop() {
#ifdef DEBUG_SAMPLER
  debug_loop();
#else
  size_t buffered_len = num_samples();
  if (buffered_len > SAMPLE_LEN / 2) {
    Serial.printf("Fell Behind\n");
    reset_sampler();
    return;
  }
  for (size_t i = 0; i < buffered_len; i++) {
    // Demodulate PPM signal
    tracker.process_sample(get_next_sample());

    // // Used for debugging
    // size_t last_count = 0;
    // if (last_count > 0 && tracker.cur_rx_len() == 0) {
    //   Serial.printf("%lu %u\n", millis(), last_count);
    // }
    // last_count = tracker.cur_rx_len();

    // If full message is received publish it and reset tracker
    if (tracker.cur_rx_len() == MSG_LEN) {
      for (int i = 0; i < MSG_BYTES; i++) {
        Serial.printf("%02X", tracker.get_msg()[i]);
      }
      Serial.print("\n");

      if (!client.connected()) {
        reconnect();
      }
      client.publish("weather_data", tracker.get_msg(), MSG_BYTES);
      decode_and_publish(tracker.get_msg());
      client.loop();
      reset_sampler();
      tracker.reset();
    }
  }
  delayMicroseconds(SAMPLE_PERIOD_US * 10);
#endif
}
