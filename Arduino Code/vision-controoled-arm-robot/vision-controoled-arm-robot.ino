#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define BASE_PIN 0
#define SHOULDER_PIN 1
#define ELBOW_PIN 2
#define END_EFFECTOR_PIN 3
#define MIN_PWM 135
#define MAX_PWM 520
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int base_angle=90, shoulder_angle=90, elbow_angle=90, grab_angle=120;
int prev_base=90, prev_shoulder=90, prev_elbow=90, prev_grab=120;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(1000);
}

void loop() {
  if(Serial.available()){
    String line = Serial.readStringUntil('\n');
    int vals[4];
    int idx = 0;
    char *token = strtok((char*)line.c_str(), ",");
    while(token && idx<4){
      vals[idx++] = atoi(token);
      token = strtok(NULL,",");
    }
    if(idx==4){
      base_angle=vals[0]; shoulder_angle=vals[1]; elbow_angle=vals[2]; grab_angle=vals[3];
    }
  }
  smoothServo(BASE_PIN, prev_base, base_angle);
  smoothServo(SHOULDER_PIN, prev_shoulder, shoulder_angle);
  smoothServo(ELBOW_PIN, prev_elbow, elbow_angle);
  smoothServo(END_EFFECTOR_PIN, prev_grab, grab_angle);
}

void smoothServo(int pin, int &current, int target){
  if(abs(target-current)<1) return;
  int step = (target-current)/5; 
  if(step==0) step = (target>current)? 1:-1;
  current += step;
  int pulse = map(current, 0, 180, MIN_PWM, MAX_PWM);
  pwm.setPWM(pin, 0, pulse);
}