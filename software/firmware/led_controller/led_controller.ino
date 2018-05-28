#include "FastLED.h"
#include <ArduinoJson.h>

////////// Pinout Variables //////////
const int led_power_status_pin = 8;
const int led_general_status_pin = 9;
const int power_on_status_pin = 7;
const int led_pwm_pin = 6;

////////// Global Variables //////////
const int num_power_status_leds = 69;
CRGB power_status_leds[num_power_status_leds];

const int num_general_status_leds = 18;
CRGB general_status_leds[num_general_status_leds];

char read_buffer[1500];

bool power_on = false;
bool was_on = false;


void setup() {
  Serial.begin(57600);

  FastLED.addLeds<WS2812, led_power_status_pin, GRB>(power_status_leds, num_power_status_leds).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<WS2812, led_general_status_pin, GRB>(general_status_leds, num_general_status_leds).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(40);

  pinMode(power_on_status_pin, INPUT);
  pinMode(led_pwm_pin, OUTPUT);

  analogWriteFrequency(led_pwm_pin, 38000);
  analogWrite(led_pwm_pin, 128);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', read_buffer, 1500);
  }

  StaticJsonBuffer<1500> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(read_buffer);

  if (root.success()) {
    Serial.println("worked!");
    JsonArray& leds = root["leds"];

    int led_count = 0;
    for (unsigned int i = 0 ; i < min(num_general_status_leds * 3, leds.size()) ; i += 3) {
      general_status_leds[led_count].red = leds[i];
      general_status_leds[led_count].green = leds[i + 1];
      general_status_leds[led_count].blue = leds[i + 2];
      led_count++;
    }
  }

  EVERY_N_MILLISECONDS(250) {
    if (digitalRead(power_on_status_pin)) {
      if (was_on) {
        fill_solid(power_status_leds, num_power_status_leds, CRGB::Black);
        was_on = 0;
      } else {
        fill_solid(power_status_leds, num_power_status_leds, CRGB::Red);
        was_on = 1;
      }
    } else {
      fill_solid(power_status_leds, num_power_status_leds, CRGB::Green);
    }
  }

  FastLED.show();
}
