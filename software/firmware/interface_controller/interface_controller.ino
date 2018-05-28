#include <Servo.h>4

////////// Pinout Variables //////////
const int led_data_pin = 9;
const int led2_data_pin = 8;
const int shooting_servo_pin = 11;
const int tamping_servo_pin = 10;
const int relay_control_pin = 3;
const int pressure_sensor_pin = A0;

const int denso_sense_pin_2 = 4;
const int denso_sense_pin_3 = 5;

const int ball_sense_pwm_pin = 7;
const int ball_sense_feedback_pin = 6;

////////// Global Variables //////////
String read_string = "";

const int safety_psi_max = 80;
float current_set_pressure = 0;
float current_actual_pressure = 0;
bool set_pressure_reached = false;
float allowed_psi_error = 4.0;

bool should_tamp = false;
bool should_shoot = false;

Servo firing_servo;
Servo tamping_servo;

void setup() {
  Serial.begin(57600);

  pinMode(pressure_sensor_pin, INPUT);
  pinMode(ball_sense_feedback_pin, INPUT);
  pinMode(denso_sense_pin_2, INPUT);
  pinMode(denso_sense_pin_3, INPUT);
  pinMode(led_data_pin, INPUT);
  pinMode(led2_data_pin, INPUT);
  pinMode(ball_sense_pwm_pin, INPUT);

  pinMode(shooting_servo_pin, OUTPUT);
  pinMode(tamping_servo_pin, OUTPUT);
  pinMode(relay_control_pin, OUTPUT);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  digitalWrite(relay_control_pin, LOW);

  firing_servo.attach(shooting_servo_pin);
  firing_servo.write(170);

  tamping_servo.attach(tamping_servo_pin);
  tamping_servo.write(180);
}

void loop() {
  check_serial_input();
  get_current_actual_pressure();
  pressurize_if_needed();
  tamp_if_needed();
  shoot_if_needed();
  print_state();
  delay(50);
}

// Set Pressure|Tamp?|Shoot?|
// XXX|X|X|
void check_serial_input() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      read_string += (char)Serial.read();
    }

    if (read_string.endsWith("\n")) {
      // Vairables to break things up
      int last_substring_breakpoint = 0;
      int current_substring_breakpoint = 0;

      // Get substring and turn into pressure
      current_substring_breakpoint = read_string.substring(last_substring_breakpoint).indexOf('|');
      current_set_pressure = read_string.substring(last_substring_breakpoint, current_substring_breakpoint + last_substring_breakpoint).toInt();
      last_substring_breakpoint += current_substring_breakpoint + 1;

      // Get whether we should tamp
      current_substring_breakpoint = read_string.substring(last_substring_breakpoint).indexOf('|');
      should_tamp = read_string.substring(last_substring_breakpoint, current_substring_breakpoint + last_substring_breakpoint).toInt();
      last_substring_breakpoint += current_substring_breakpoint + 1;


      // Get whether we should shoot
      current_substring_breakpoint = read_string.substring(last_substring_breakpoint).indexOf('|');
      should_shoot = read_string.substring(last_substring_breakpoint, current_substring_breakpoint + last_substring_breakpoint).toInt();
      last_substring_breakpoint += current_substring_breakpoint + 1;

      read_string = "";
    }

  }
}

void get_current_actual_pressure() {
  current_actual_pressure = read_pressure();
}

void pressurize_if_needed() {
  static bool keep_charging = false;

  if (current_set_pressure > 0) {

    if (keep_charging) {
      digitalWrite(relay_control_pin, HIGH);

      if (current_actual_pressure >= current_set_pressure) {
        keep_charging = false;
        digitalWrite(relay_control_pin, LOW);
      }
    }

    if ((current_set_pressure - current_actual_pressure) > allowed_psi_error) {
      keep_charging = true;
    }

  } else {
    digitalWrite(relay_control_pin, LOW);
  }
}

void tamp_if_needed() {
  if (should_tamp) {

    tamping_servo.write(60);

    delay(1000);
    tamping_servo.write(180);

    should_tamp = false;
  }
}

void shoot_if_needed() {
  if (should_shoot) {
    firing_servo.write(100);
    delay(500);
    firing_servo.write(170);

    current_set_pressure = 0;
    should_shoot = false;
  }
}

void print_state() {
  Serial.print(current_actual_pressure);
  Serial.print("|");
  Serial.print(get_set_pressure());
  Serial.print("|");
  Serial.print(get_compressor_on());
  Serial.print("|");
  Serial.print(ball_detected());
  Serial.print("|");
  Serial.print(arm_motor_power_on());
  Serial.println();
}

bool arm_motor_power_on() {
  return digitalRead(denso_sense_pin_2);
}

bool ball_detected() {
  return digitalRead(ball_sense_feedback_pin);
}

bool get_compressor_on() {
  return digitalRead(relay_control_pin);
}

float get_set_pressure() {
  return current_set_pressure;
}

float read_pressure() {
  analogRead(pressure_sensor_pin);
  uint16_t raw_adc = analogRead(pressure_sensor_pin);

  float voltage = 0.004887585532746823 * (float) raw_adc;  //ADC count (10 bit) to voltage conversion

  float pressure = (250000.0 * voltage - 250000.0) * 0.000145038; //Convert voltage to pascals, then to PSI

  return max(pressure, 0);
}
