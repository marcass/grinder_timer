//Runs a mazzer superjolly grinder with timer
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

#define debug

/* FINITE STATE MACHINE STATES
 *  STATE_IDLE -> STATE_GRINDING
 *  STATE_DONE -> STATE_IDLE
 *  STATE GRINDING -> STATE_DONE (timer mode)
 *  STATE_GRINDING -> STATE_IDLE (on demand mode)
 *  
 */
 
const int STATE_IDLE = 0;
const int STATE_GRINDING = 1;
const int STATE_DONE = 2;
int state = STATE_IDLE;

// this constant won't change:

const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
const int NUM_ADC_STATES = 1024;

const int GRIND_INT = 0;  // 0 = digital pin 2
const int GRIND_BUTTON = 2;
const int POTI_PIN = 3;    // select the input pin for the potentiometer
const int RELAY_PIN = 13;
//button for changing grind mode (timer or on push) or stopping a timed grind
const int EVENT_BUTTON = 8; //or any button that is convenient
unsigned long LAST_DEBOUNCE_TIME = 0;  // the last time the output pin was toggled
unsigned long DEBOUNCE_DELAY = 50;    // the debounce time; increase if the output flickers
//grinder mode. if timer is true it is timer! If false it is on demand grinding on button press
boolean timer_mode = true;

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int lastGrindButtonState = 0;
int GrindButtonState = 0;
unsigned long grind_start = 0;
unsigned long grind_time;
unsigned long now;

int val = 0;
char buf[16];
int grind_time_preset = MIN_GRIND_TIME;

int sensorValue = 0;  // variable to store the value coming from the potentiaometer
int sensorValueNew = 0;

void setup() {
  // initialize the button pin as a input:
  pinMode(GRIND_BUTTON, INPUT);
  digitalWrite(GRIND_BUTTON, HIGH);
 
  // initialize the RELAY as an output:
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  //initialize event button
  pinMode(EVENT_BUTTON, INPUT);
  digitalWrite(EVENT_BUTTON, HIGH); //check to see if it needs to be pulled up or down
  // initialize serial communication:
  Serial.begin(9600);  // Used to type in characters
  // initialize lcd
  lcd.begin(16,2);
  lcd.backlight(); // finish with backlight on  
  lcd.clear();
  //attach interrupt for grind button
  attachInterrupt(GRIND_INT, grinding, FALLING); //check to see if wired falling or rising
}


void loop() {
  switch (state) {
  case STATE_IDLE:
    proc_idle();
    break;
  case STATE_GRINDING:
    proc_grinding();
    break;
  case STATE_DONE:
    proc_done();
    break;
  }
  manage_outputs();
  update_display();
}

void proc_idle(){
  grind_start = 0;
  if (timer_mode){
    attachInterrupt(GRIND_INT, grinding, FALLING); //check to see if wired falling or rising
  }
  #ifdef debug
    Serial.print("State is ");
    Serial.print(state);
    Serial.print(": timer_mode: ");
    Serial.println(timer_mode);
  #endif

  /*
   * Need to put stuff in here for grinding when in demand mode (!=timer_mode)
   * So when button is pressed we enter grinding state. Must ride bike now
   */
   if (!timer_mode){
     int slap = digitalRead(GRIND_BUTTON); 
     if (slap == LOW){
       //do debounce stuff
       //set button state to look for changes
       if (slap != lastGrindButtonState){
         // reset the debouncing timer     
         LAST_GRIND_DEBOUNCE_TIME = millis();
       }
       if (millis() - LAST_GRIND_DEBOUNCE_TIME > debounceDelay) {
        state = STATE_GRINDING;
       }
     }else{
      state = STATE_IDLE;
     }
     if (slap != GrindButtonState){
      GrindButtonState = slap);
     }
   }
  
  // read the state of the switch into a local variable:
  int event = digitalRead(EVENT_BUTTON);
  // check to see if you just pressed the button
  // (i.e. the input went from HIGH to LOW),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (event != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (event != buttonState) {
      buttonState = event;

      // only toggle the grinder mode if the new button state is LOW
      if (buttonState == LOW) {
        timer_mode = !timer_mode;
      }
    }
  }
  //do nothing
}

void proc_grinding(){
  #ifdef debug
    Serial.println(state);
  #endif
  switch(timer_mode) {
    case false:
      //grind on demand so while button is pushed we will grind
      break;
    case true:
      timer_grinding();
      break;
  }


void timer_grinding(){  
  detachInterrupt(GRIND_BUTTON);
  if (grind_start == 0){
    grind_start = millis();
    lcd.clear();
  }
  now = millis();
  grind_time = now - grind_start;
  
  if (grind_time > grind_time_preset){
    state = STATE_DONE;
  }else{
    //do nothing
    #ifdef debug
      Serial.print("Grind time = ");
      Serial.print(grind_time);
      Serial.println("s");
    #endif
  }
}

void proc_done(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.write("Done.");
    lcd.setCursor(0,1);
    lcd.write("cool down..");
    delay(3000);//delay of 3s for accidental repushing button
    state = STATE_IDLE;
    lcd.clear();
}

void grinding(){
  state = STATE_GRINDING;
}

void manage_outputs(){
  if (state == STATE_GRINDING){
    digitalWrite(RELAY_PIN, HIGH);
  }else{
    digitalWrite(RELAY_PIN, LOW);
  }
}

void update_display(){
  if (state == STATE_GRINDING){
    lcd.setCursor(0,0);
    lcd.write("grinding...");
    int percent = 100*grind_time/grind_time_preset;
    int bar_frac = 16*grind_time/grind_time_preset;
    sprintf(buf, "%3d %", percent);
    lcd.write(buf);
    lcd.setCursor(0,1);
    for (int x = 0; x < 16; x++) {
      if (x < bar_frac) {      lcd.write(3);}
      else {lcd.write(20);}
    }

  }else{
    val = analogRead(POTI_PIN);
    grind_time_preset =  int(float(val)/(NUM_ADC_STATES-1) * (MAX_GRIND_TIME-MIN_GRIND_TIME)) + MIN_GRIND_TIME;
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.write("Grinder Control");
    lcd.setCursor(4,1);
    sprintf(buf, "%5d ms", grind_time_preset);
    lcd.write(buf);
  }
}







