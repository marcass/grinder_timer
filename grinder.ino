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
 
const int STATE_IDLE = 0;
const int STATE_GRINDING = 1;
const int STATE_DONE = 2;

const int MODE_TIMER = 0;
const int MODE_DEMAND = 1;

volatile int state = STATE_IDLE;
volatile int mode = MODE_TIMER;

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
const int STATUS_LED_PIN = 9; // pin9 is a PWM pin and allows for analogWrite.
//switching phase and neutral for safety
const int RELAY_PIN_L = 11;
const int RELAY_PIN_N = 12;
//button for changing grind mode (timer or on push) or stopping a timed grind
const int DEBOUNCE_DELAY = 50;    // the debounce time; increase if the output flickers

// Variables will change:
unsigned long grind_start = 0;
unsigned long grind_time = 0;
unsigned long now;
unsigned long grind_debounce_time = 0;
int status_led_brightness = 0;
int fade_rate = 10;

//interrupt hell! setting flags to keep them short and update them in idle if not set
volatile bool grind_interrupt = 1;
volatile bool event_interrupt = 0;
volatile bool mode_interrupt = 1; //on event button but names are confusing

int val = 0;
char buf[16];
int grind_time_preset = MIN_GRIND_TIME;

int sensorValue = 0;  // variable to store the value coming from the potentiaometer
int sensorValueNew = 0;

void setup() {
  // initialize the button pin as an input
  pinMode(GRIND_BUTTON, INPUT);
  digitalWrite(GRIND_BUTTON, LOW);
  // initialize the button LED as an output
  pinMode(STATUS_LED_PIN, OUTPUT);
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
  attachInterrupt(EVENT_INT, mode_change, RISING);
}

void mode_change() {
  if (mode == MODE_TIMER) {
    
    mode = MODE_DEMAND;
  }
  else if (mode == MODE_DEMAND) {
    state = STATE_DONE;
    mode = MODE_TIMER;
  }
}

void grinding(){
//  detachInterrupt(GRIND_INT);
//  grind_interrupt = 0;
  detachInterrupt(EVENT_INT);
  mode_interrupt = 0;
  attachInterrupt(EVENT_INT, stop_grinding, RISING);
  event_interrupt = 1;
  state = STATE_GRINDING;
  //delay(50);
}

//  attachInterrupt(GRIND_INT, grinding, RISING); //check to see if wired falling or rising
//  attachInterrupt(EVENT_INT, mode_change, RISING);
//  attachInterrupt(EVENT_INT, stop_grinding, RISING);

long int new_val;


void proc_idle() {
  val = analogRead(POTI_PIN);
  grind_time_preset =  int(float(val)/(NUM_ADC_STATES-1) * (MAX_GRIND_TIME-MIN_GRIND_TIME)) + MIN_GRIND_TIME;
  if (abs(val-new_val) > 20 ) {
    update_display();
    new_val = val;
  }

  //interupts not working for mode so read value of button
//  if (digitalRead(EVENT_BUTTON) == HIGH) { 
//    //do debounce stuff
//    if (grind_debounce_time == 0){
//      grind_debounce_time = millis();
//    }
//    if (millis() - grind_debounce_time > DEBOUNCE_DELAY) {
//      //over debounce threshold so change mode
//      if (mode == MODE_TIMER) {
//        detachInterrupt(GRIND_INT);
//        grind_interrupt = 0;
//        delay(1000);
//        mode = MODE_DEMAND;
//        //delay(1000);
//        grind_debounce_time = 0;
//      }
//      if (mode == MODE_DEMAND) {
//        attachInterrupt(GRIND_INT, grinding, RISING);
//        grind_interrupt = 1;
//        delay(1000);
//        mode = MODE_TIMER;
//        delay(1000);
//        grind_debounce_time = 0;
//      }
//       update_display();
//    }
//  }
  //sanity chekc for failing code
  if (mode == MODE_TIMER) {
//    if (!grind_interrupt) {
//      attachInterrupt(GRIND_INT, grinding, RISING);
//      grind_interrupt = 1;
//    }
    //CLEAR EVENT ITERUPT FIRST
    if (event_interrupt) {
      detachInterrupt(EVENT_INT);
      event_interrupt = 0;
      attachInterrupt(EVENT_INT, mode_change, RISING);  
      mode_interrupt = 1; 
    }
    if (digitalRead(GRIND_BUTTON) == HIGH) { 
      //do debounce stuff
      if (grind_debounce_time == 0){
        grind_debounce_time = millis();
      }
      if (millis() - grind_debounce_time > DEBOUNCE_DELAY) {
        //over debounce threshold so change state
        state = STATE_GRINDING;
        update_display();
      }
    }
  }
  
  //if not in timer mode need to grind on button push
  //Interrupt detached in mode change
  if (mode == MODE_DEMAND) {
    //sanity check for failing code
//    if (grind_interrupt) {
//      detachInterrupt(GRIND_INT);
//      grind_interrupt = 0;
//    }
    if (event_interrupt) {
      detachInterrupt(EVENT_INT);
      event_interrupt = 0;
    }
    if (digitalRead(GRIND_BUTTON) == HIGH) { 
      //do debounce stuff
      if (grind_debounce_time == 0){
        grind_debounce_time = millis();
      }
      if (millis() - grind_debounce_time > DEBOUNCE_DELAY) {
        //over debounce threshold so change state
        state = STATE_GRINDING;
        update_display();
      }
    }
  }
  //for state change while in idle
  if (!mode_interrupt) {
    //get rid of  grinding event interrupt if it is there
    if (event_interrupt) {
      detachInterrupt(EVENT_INT);
      event_interrupt = 0;
    }
    attachInterrupt(EVENT_INT, mode_change, RISING);  
    mode_interrupt = 1;  
  }
}

void stop_grinding(){
  //lcd.clear();
  //update_display();
  state = STATE_DONE;
  detachInterrupt(EVENT_INT);
  event_interrupt = 0;
}

void proc_grinding(){
  #ifdef debug
    Serial.println(state);
    Serial.println("Grinding!!!");
  #endif
  update_display();
  if (mode == MODE_TIMER) {
     #ifdef debug
        Serial.print("Grind time = ");
        Serial.print(grind_time);
        Serial.println("s");
    #endif
    if (grind_start == 0){
      grind_start = millis();
    }
    now = millis();
    grind_time = now - grind_start;   //grinding ends if grind time reached or event button is pressed to cancel
    if (grind_time > grind_time_preset){
      //handle a cancellation push of button with interrupt on event button
      lcd.clear();
      update_display();
      state = STATE_DONE;
    }
  } else if (mode == MODE_DEMAND) {   //grind on demand so while button is pushed we will grind
    if (digitalRead(GRIND_BUTTON) == LOW){
      state = STATE_IDLE;
      lcd.clear();
      update_display();
    }
  }
}

void proc_done(){
  update_display();
  grind_start = 0;
  if (digitalRead(GRIND_BUTTON) == LOW) {
     manage_outputs();
     if (mode == MODE_TIMER) {
       delay(COOL_DOWN);
       detachInterrupt(EVENT_INT);
//       event_interrupt = 0;
//       attachInterrupt(GRIND_INT, grinding, RISING);
//       grind_interrupt = 1;
       attachInterrupt(EVENT_INT, mode_change, RISING);
       mode_interrupt = 1;
       state = STATE_IDLE;
     }
     if (mode == MODE_DEMAND) {
       state = STATE_IDLE;
     }
     lcd.clear();
     //delay(1000);
     update_display();
  }
}

void manage_outputs(){
  if (state == STATE_GRINDING){
    digitalWrite(RELAY_PIN_L, HIGH);
    digitalWrite(RELAY_PIN_N, HIGH);
    analogWrite(STATUS_LED_PIN, status_led_brightness);
    status_led_brightness += fade_rate;    
    if (status_led_brightness <= 0 || status_led_brightness >= 255) {
      fade_rate = -fade_rate;
    }
    //delay(1); // delay to see fade
  }else{
    digitalWrite(RELAY_PIN_L, LOW);
    digitalWrite(RELAY_PIN_N, LOW);
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
}


void update_display(){
  int bar_frac;
  switch (state) { 
    case STATE_GRINDING:
      lcd.setCursor(0,0);
      lcd.write("grinding...");
      bar_frac = 16*grind_time/grind_time_preset;
      if (mode==MODE_TIMER) {
        lcd.write(buf);
        lcd.setCursor(0,1);
        for (int x = 0; x < 16; x++) {
          if (x < bar_frac) {      lcd.write(3);}
          else {lcd.write(20);}
        }
      }
      break;
    case STATE_IDLE:
      lcd.setCursor(0,0);
      if (mode == MODE_DEMAND){
        lcd.write("Demand");
      }else{
        lcd.write("Timer  ");
      }
      lcd.setCursor(4,1);
      sprintf(buf, "%5d ms", grind_time_preset);
      lcd.write(buf);
      break;
    case STATE_DONE:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write("Done.");
      lcd.setCursor(0,1);
      lcd.write("cool down..");
      break;
    }
}


void loop() {
  manage_outputs();
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
  #ifdef debug
    Serial.print("State is ");
    Serial.print(state);
    Serial.print("   mode change int is: ");
    Serial.print(mode_interrupt);
    Serial.print("     Mode is: ");
    Serial.println(mode);
   #endif
}









