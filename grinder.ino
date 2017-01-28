//Runs a mazzer superjolly grinder with timer
#include <Wire.h>
//https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

#define debug

/* FINITE STATE MACHINE STATES
 *  STATE_IDLE_TIMER -> STATE_GRINDING || STATE_IDLE_DEMAND
 *  STATE_IDLE_DEMAND -> STATE_GRINDING || STATE_IDLE_TIMER
 *  STATE_DONE -> STATE_IDLE_DEMAND || STATE_IDLE_TIME
 *  STATE GRINDING -> STATE_DONE 
 *  
 *  
 */
 
const int STATE_IDLE_TIMER = 0;
const int STATE_GRINDING = 1;
const int STATE_DONE = 2;
const int STATE_IDLE_DEMAND = 3;
int state = STATE_IDLE_TIMER;

// this constant won't change:

const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
const int NUM_ADC_STATES = 1024;
const int COOL_DOWN = 3000; //number of seconds to cool down after grinding

const int GRIND_INT = 0;  // 0 = digital pin 2
const int GRIND_BUTTON = 2;
const int EVENT_BUTTON = 3; //ned interrupt on this button too
const int EVENT_INT = 1; //digital pin 3
const int POTI_PIN = 3;    // select the input pin for the potentiometer analogue pin 3
//switching phase and neutral for safety
const int RELAY_PIN_L = 11;
const int RELAY_PIN_N = 12;
//button for changing grind mode (timer or on push) or stopping a timed grind
const int DEBOUNCE_DELAY = 50;    // the debounce time; increase if the output flickers

// Variables will change:
unsigned long grind_start = 0;
unsigned long cool_start = 0;
unsigned long cool_time = 0;
unsigned long grind_time;
unsigned long now;
unsigned long GRIND_DEBOUNCE_TIME = 0;
int prev_state = 0; //place holder for state to exit state_doe to

int val = 0;
char buf[16];
int grind_time_preset = MIN_GRIND_TIME;

int sensorValue = 0;  // variable to store the value coming from the potentiaometer
int sensorValueNew = 0;

void grinding(){
  state = STATE_GRINDING;
}

void setup() {
  // initialize the button pin as a input:
  pinMode(GRIND_BUTTON, INPUT);
  digitalWrite(GRIND_BUTTON, LOW);
 
  // initialize the RELAY as an output
  pinMode(RELAY_PIN_N, OUTPUT);
  digitalWrite(RELAY_PIN_N, LOW);
  pinMode(RELAY_PIN_L, OUTPUT);
  digitalWrite(RELAY_PIN_L, LOW);  
  //initialize event button
  pinMode(EVENT_BUTTON, INPUT);
  digitalWrite(EVENT_BUTTON, LOW); //check to see if it needs to be pulled up or down
  // initialize serial communication:
  Serial.begin(115200);  // Used to type in characters
  // initialize lcd
  lcd.begin(16,2);
  lcd.backlight(); // finish with backlight on  
  lcd.clear();
  //attach interrupt for grind button
  //attachInterrupt(GRIND_INT, grinding, RISING); //check to see if wired falling or rising
}

void state_change() {
  detachInterrupt(EVENT_BUTTON);
  switch (prev_state) {
    case STATE_IDLE_TIMER:
      state = STATE_IDLE_DEMAND;
      break;
    case STATE_IDLE_DEMAND:
       state = STATE_IDLE_TIMER;
      break;
  }
}

//need to place functions above calls in new arduino ide
void proc_idle_timer(){
  attachInterrupt(GRIND_INT, grinding, RISING); //check to see if wired falling or rising
  //set this as state to return to
  prev_state = state;
  // To change grinding mode in idle read the state of the switch into a local variable:
  
  
  //for state change while in idle
  attachInterrupt(EVENT_INT, state_change, RISING);
}

void proc_idle_demand() {
  detachInterrupt(GRIND_BUTTON);
  //if not in timer mode need to grind on button push
  prev_state = state;
  if (digitalRead(GRIND_BUTTON) == HIGH) { 
  
    //do debounce stuff
    if (GRIND_DEBOUNCE_TIME == 0){
      GRIND_DEBOUNCE_TIME = millis();
    }
    if (millis() - GRIND_DEBOUNCE_TIME > DEBOUNCE_DELAY) {
      //over debounce threshold so change state
      state = STATE_GRINDING;
    }
  }
  //for state change while in idle
  attachInterrupt(EVENT_INT, state_change, RISING);      
}

void stop_grinding(){
  state = STATE_DONE;
  
}

void timer_grinding(){  
  cool_start = 0;
  detachInterrupt(GRIND_BUTTON);
    if (grind_start == 0){
    grind_start = millis();
    lcd.clear();
  }
  now = millis();
  grind_time = now - grind_start;
  attachInterrupt(EVENT_INT, stop_grinding, RISING);
  //grinding ends if grind time reached or event button is pressed to cancel
  if (grind_time > grind_time_preset){
    //handle a cancellation push of button with interrupt on event button
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

void proc_grinding(){
  #ifdef debug
    Serial.println(state);
  #endif
  if (prev_state == STATE_IDLE_TIMER) {
     timer_grinding();
  }
  //grind on demand so while button is pushed we will grind
  if (prev_state == STATE_IDLE_DEMAND) {
    if (digitalRead(GRIND_BUTTON) == LOW){
      state = STATE_DONE;
    }else{
      #ifdef debug
      Serial.println("Demand grinding!!!");
      #endif
    }
  }
}

void proc_done(){
    grind_start = 0;
    detachInterrupt(EVENT_BUTTON);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.write("Done.");
    lcd.setCursor(0,1);
    lcd.write("cool down..");
    //delay doesn't work here as relays stay high so need to do an async time test
    if (digitalRead(GRIND_BUTTON) == LOW) {
       manage_outputs();
       delay(COOL_DOWN);
       lcd.clear();
       state = prev_state;
    }else {
      state = STATE_DONE;
    }
}

void manage_outputs(){
  if (state == STATE_GRINDING){
    digitalWrite(RELAY_PIN_L, HIGH);
    digitalWrite(RELAY_PIN_N, HIGH);
  }else{
    digitalWrite(RELAY_PIN_L, LOW);
    digitalWrite(RELAY_PIN_N, LOW);
  }
}

void update_display(){
  if (state == STATE_GRINDING){
    lcd.setCursor(0,0);
    lcd.write("grinding...");
    //int percent = 100*grind_time/grind_time_preset;
    int bar_frac = 16*grind_time/grind_time_preset;
    //sprintf(buf, "%3d %", percent);
    switch (prev_state) {
      case STATE_IDLE_TIMER:
        lcd.write(buf);
        lcd.setCursor(0,1);
        for (int x = 0; x < 16; x++) {
          if (x < bar_frac) {      lcd.write(3);}
          else {lcd.write(20);}
        }
        break;
      case STATE_IDLE_DEMAND:
         lcd.setCursor(0,1);
         lcd.write("              ");
         break;
    }
  }else{
    val = analogRead(POTI_PIN);
    grind_time_preset =  int(float(val)/(NUM_ADC_STATES-1) * (MAX_GRIND_TIME-MIN_GRIND_TIME)) + MIN_GRIND_TIME;
    //lcd.clear();
    lcd.setCursor(0,0);
    if (prev_state == STATE_IDLE_DEMAND){
      lcd.write("Demand");
    }else{
      lcd.write("Timer  ");
    }
    lcd.setCursor(4,1);
    sprintf(buf, "%5d ms", grind_time_preset);
    lcd.write(buf);
  }
}


void loop() {
  switch (state) {
  case STATE_IDLE_TIMER:
    proc_idle_timer();
    break;
  case STATE_IDLE_DEMAND:
    proc_idle_demand();
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
 #ifdef debug
  Serial.print("State is ");
  Serial.print(state);
  Serial.print("   Cool time = ");
  Serial.print(cool_time);
  Serial.print("     Preivious state is: ");
  Serial.println(prev_state);
 #endif 
  
}









