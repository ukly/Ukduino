#include <Servo.h>
#define PIN_SERVO 10 // [3110] 10번핀 서보 연결
#define PIN_IR A0 //[3104] 적외선 거리센서 PIN - Analog0 정의
#define PIN_LED 9

// Framework setting
#define _DIST_TARGET 255 //[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값
#define _DIST_MAX 410 //[3117] 거리 최대값

// Distance sensor
#define _DIST_ALPHA 0.35  //[3099] EMA 필터링을 위한 alpha 값
// [3108] 0~1 사이의 값

// Servo range
#define _DUTY_MIN 1071
#define _DUTY_NEU 871
#define _DUTY_MAX 671

// Servo speed control
#define _SERVO_ANGLE 25
#define _SERVO_SPEED 100
// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 1.2
#define _KD 54.5
#define _KI 0.01455
  
#define a 70
#define b 370
//////////////////////
// global variables //
//////////////////////
#define DELAY_MICROS 1500

float samples_num = 3;

// Servo instance
Servo myservo;

// Distance sensor
float dist_target = 255; // location to send the ball
float dist_raw, dist_ema; //[1928] 측정된 값과 ema 필터를 적용한 값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
//[3104] 각 event의 진행 시간 저장 변수
bool event_dist, event_servo, event_serial;
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

const float coE[] = {-0.0000272, 0.0187197, -2.7226322, 246.3408758};


void setup() {
  // initialize GPIO pins for LED and attach servo
  //myservo.attach(PIN_SERVO); // attach servo
  //myservo.writeMicroseconds(_DUTY_NEU);

  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = millis();
  
  pinMode(PIN_LED, OUTPUT); // initialize GPIO pins

  // initialize global variables

  // initialize serial port
  Serial.begin(57600);

  // move servo to neutral position
  //myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;
  duty_target = _DUTY_NEU;

  //reset iterm
  dterm = pterm = 0;
  iterm = 3;
 

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / _SERVO_ANGLE*2 ) * (_INTERVAL_SERVO / 1000.0);

  myservo.attach(PIN_SERVO); // attach servo
  myservo.writeMicroseconds(_DUTY_NEU);
  
}


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////

  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }

  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }

  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
    dist_raw = filtered_ir_distance();

    


    // PID control logic
    error_curr = _DIST_TARGET - dist_raw;
    pterm = _KP * error_curr;
    iterm += _KI * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm + iterm + dterm;
    duty_target = _DUTY_NEU + control;


    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MIN) {
      duty_target = _DUTY_MIN;
    }
    else if (duty_target < _DUTY_MAX) {
      duty_target = _DUTY_MAX;
    }

    error_prev = error_curr;

  }

  if (event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  //myservo.writeMicroseconds(_DUTY_NEU);

  if (event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
   
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return 300.0 / (b - a) * (val - a) + 100;

}
//[3099]

float under_noise_filter(void) {
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) {
      largestReading = currReading;
    }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void) {
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) {
      lowestReading = currReading;
    }
  }
  dist_ema = _DIST_ALPHA * lowestReading + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}
