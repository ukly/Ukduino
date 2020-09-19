#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 1;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); //toggle LED value
  digitalWrite(PIN_LED, toggle); // update LED status
  delay(1000); // wait for 1,000 milliseconds
  for(int cnt=0; cnt<=10; ++cnt){    
      toggle = toggle_state(toggle);
      digitalWrite(PIN_LED, toggle);
      delay(100);
  }
  while(1){}
}

 int toggle_state(int toggle) {
  if (toggle == 0) {
    toggle = 1;
  } else {
    toggle = 0;
  }
  return toggle;
 }
