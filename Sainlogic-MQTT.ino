#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <ESP8266TimerInterrupt.h>
#include <PubSubClient.h>

#include "ring_buffer.h"
#include "ppm_tracker.h"
#include "data_decode.h"
#include "secrets.h"

#include <time.h>

#define NTP_SERVER "pl.pool.ntp.org"
#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"

time_t now;  // this are the seconds since Epoch (1970) - UTC
tm tm;       // the structure tm holds time information in a more convenient way

int saved_day;
int saved_minute;

float rain_start_1h;
float rain_start_24h;

struct rain {
  float rain_1h;
  float rain_24h;
};
typedef struct rain Struct;

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
const char* mqtt_server = "broker.hivemq.com";

WiFiClient wifi_client;
PubSubClient client(wifi_client);

// Init ESP8266 timer
ESP8266Timer ITimer;

BinaryPpmTracker tracker(MIN_SAMPLES, MAX_SAMPLES);

uint32_t sntp_startup_delay_MS_rfc_not_less_than_60000 () {
  return 10000UL; // 10s
}

uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 () {
  return 60 * 1000UL; // 1 minute
}
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
    String clientId = "ZeatynisESP8266-";
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

  configTime(TIMEZONE, NTP_SERVER);
  Serial.println("Waiting for NTP data to be pulled");
  delay(15000); // for NTP to pull the data
  time(&now);
  localtime_r(&now, &tm);

  saved_day = tm.tm_yday;
  saved_minute = tm.tm_min;

  Serial.printf("Saved day: %u\n", saved_day);
  Serial.printf("Saved minute: %u\n", saved_minute);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //client.setSocketTimeout(0xFFFF);
  client.setKeepAlive(0xFFFF);

  if (ITimer.attachInterruptInterval(SAMPLE_PERIOD_US, sample_input))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or interval");
}

Struct calculate_rain(float rain_measure) {
  time(&now);
  localtime_r(&now, &tm);
  Struct rain;
  if (tm.tm_yday == 0 && saved_day > 0 || tm.tm_yday > saved_day) {
    // reset 24h rain every 24h
    rain_start_24h = rain_measure;
  }
  if (tm.tm_min < saved_minute) {
    // reset 1h rain every hour
    rain_start_1h = rain_measure;
  }

  if (rain_measure < rain_start_24h) {
    // handle overflow on 24h rain counter
    rain.rain_24h = 9999 - rain_start_24h + rain_measure;
  } else
    rain.rain_24h = rain_measure - rain_start_24h;
  if (rain_measure < rain_start_1h) {
    // handle overflow on 1h rain counter
    rain.rain_24h = 9999 - rain_start_24h + rain_measure;
  } else
    rain.rain_1h = rain_measure - rain_start_1h;

  return rain;
}

void decode_and_publish(const uint8_t* msg) {
  if (check_crc(tracker.get_msg())) {
    Struct rain;
    rain = calculate_rain(get_rain(msg));
    String json_data = "{";
    json_data += String("\"temp_c\": ") + String(get_temperature(msg)) + ", ";
    json_data += String("\"humidity_%\": ") + String(get_humidity(msg)) + ", ";
    json_data += String("\"wind_dir_deg\": ") + String(get_direction(msg)) + ", ";
    json_data += String("\"avr_wind_m/s\": ") + String(get_avr_wind_speed(msg)) + ", ";
    json_data += String("\"gust_wind_m/s\": ") + String(get_gust_wind_speed(msg)) + ", ";
    json_data += String("\"rain_mm_1h\": ") + String(rain.rain_1h) + ", ";
    json_data += String("\"rain_mm_24h\": ") + String(rain.rain_24h);
    json_data += "}";
    Serial.print(json_data);
    Serial.print('\n');
    client.publish("Zeatynis_weather_decoded", json_data.c_str(), json_data.length());
  } else
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

    // Used for debugging
    size_t last_count = 0;
    if (last_count > 0 && tracker.cur_rx_len() == 0) {
      Serial.printf("%lu %u\n", millis(), last_count);
    }
    last_count = tracker.cur_rx_len();

    // If full message is received publish it and reset tracker
    if (tracker.cur_rx_len() == MSG_LEN) {
      for (int i = 0; i < MSG_BYTES; i++) {
        Serial.printf("%02X", tracker.get_msg()[i]);
      }
      Serial.print("\n");

      if (!client.connected()) {
        reconnect();
      }
      client.publish("Zeatynis_weather_data", tracker.get_msg(), MSG_BYTES);
      decode_and_publish(tracker.get_msg());
      client.loop();
      reset_sampler();
      tracker.reset();
    }
  }
  delayMicroseconds(SAMPLE_PERIOD_US * 10);
#endif
}