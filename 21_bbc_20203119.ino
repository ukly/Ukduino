// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

#include <Servo.h>
Servo myservo;

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1500);

// initialize serial port
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
  
  a = 78;
  b = 360;
}

  float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:100,max:450,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali < 255) {
    myservo.writeMicroseconds(1650);
    Serial.println(1650);
  }
  else {
    myservo.writeMicroseconds(1350);
    Serial.println(1350);
  }
  delay(20);
}
