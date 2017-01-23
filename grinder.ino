//Runs a mazzer superjully grinder with timer

#define debug

/* FINITE STATE MACHINE STATES
 *  state_idle -> state_grinding
 *  state grinding -> state_idle
 */
const int state_idle = 0;
const int state_grinding = 1;
int state = state_idle;

// this constant won't change:
const int grind_int = 0;  // 0 = digital pin 2
const int grind_button = 2;
const int relay = 3;
//const int  buttonPin = 2;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
unsigned long GRIND_START = 0;
unsigned long GRIND_TIME;
unsigned long now;
const int GRIND_PRESET = 9000; //9s Need to set via rotary dial and disply as integer of s to 1 deciaml place

void setup() {
  // initialize the button pin as a input:
  pinMode(grind_button, INPUT);
  digitalWrite(grind_button, HIGH);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
  Serial.begin(115200);
  //attach interrupt for grind button
  attachInterrupt(grind_int, grinding, FALLING); //check to see if wired falling or rising
}


void loop() {
  switch (state) {
  case state_idle:
    proc_idle();
    break;
  case state_grinding:
    proc_grinding();
    break;
 }
  manage_outputs();
}

void proc_idle(){
  #ifdef debug
    Serial.println(state);
  #endif
  //do nothing
}

void proc_grinding(){
  #ifdef debug
    Serial.println(state);
  #endif
  detachInterrupt(grind_button);
  if (GRIND_START == 0){
    GRIND_START = millis();
  }else{
    now = millis();
    GRIND_TIME = now - GRIND_START;
  }
  if (GRIND_TIME > GRIND_PRESET){
    state = state_idle;
    delay(3000);//delay of 3s for accidental repushing button
    attachInterrupt(grind_int, grinding, FALLING); //check to see if wired falling or rising
  }else{
    //do nothing
    #ifdef debug
      Serial.print("Grind time = ");
      Serial.print(GRIND_TIME);
      Serial.println("s");
    #endif
  }
}

void grinding(){
  state = state_grinding;
}

void manage_outputs(){
  if (state == state_grinding){
    digitalWrite(relay, HIGH);
  }else{
    digitalWrite(relay, LOW);
  }
}









