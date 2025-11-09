#include <Servo.h>

Servo myservo;

const int IR_PIN = A0;
const int LED_PIN = 9;
const int SERVO_PIN = 10; 

const long BAUD_RATE = 1000000;
const int INTERVAL = 20;
unsigned long last_time = 0;

const float DIST_MIN = 10.0;
const float DIST_MAX = 25.0;

const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

const int DUTY_MIN = 544;
const int DUTY_MAX = 2400;

float alpha = 0.2;
float dist_raw = 0;
float dist_ema = 0;
int a_value = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  myservo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);

  a_value = analogRead(IR_PIN);
  dist_raw = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;
  dist_ema = dist_raw;
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_time < INTERVAL) {
    return;
  }
  last_time = current_time;

  a_value = analogRead(IR_PIN);
  dist_raw = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;
  dist_ema = alpha * dist_raw + (1.0 - alpha) * dist_ema;

  int servo_degree = 0;
  
  if (dist_ema >= DIST_MIN && dist_ema <= DIST_MAX) {
    digitalWrite(LED_PIN, HIGH);
    servo_degree = SERVO_MIN + (dist_ema - DIST_MIN) * (SERVO_MAX - SERVO_MIN) / (DIST_MAX - DIST_MIN);
  } else if (dist_ema < DIST_MIN) {
    digitalWrite(LED_PIN, LOW);
    servo_degree = SERVO_MIN;
  } else {
    digitalWrite(LED_PIN, LOW);
    servo_degree = SERVO_MAX;
  }

  int duty = DUTY_MIN + (servo_degree - SERVO_MIN) * (float)(DUTY_MAX - DUTY_MIN) / (SERVO_MAX - SERVO_MIN);
  myservo.writeMicroseconds(duty);

  Serial.print("DUTY_MIN:"); Serial.print(DUTY_MIN);
  Serial.print("DIST_MIN:"); Serial.print(DIST_MIN);
  Serial.print("IR:"); Serial.print(a_value);
  Serial.print("dist_raw:"); Serial.print(dist_raw);
  Serial.print("ema:"); Serial.print(dist_ema);
  Serial.print("servo:"); Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(DUTY_MAX);
  Serial.println("");
}
