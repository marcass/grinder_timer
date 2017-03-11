//Runs a mazzer superjolly grinder with timer
#include <Wire.h>
//https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//preset storage (from reboot for rotary encoder value)
// PRESET STORAGE
#include <EEPROM.h>
#include "/home/mw/git/grinder/EEPROMAnything.h"
struct config_t
{
   int preset;
} configuration;

#define debug
//#define rotary_encoder
#define pot

/* FINITE STATE MACHINE STATES
 *  STATE_IDLE -> STATE_GRINDING
 *  STATE_DONE -> STATE_IDLE
 *  STATE GRINDING -> STATE_DONE (mode timer)
 *  STATE GRINDING -> STATE_IDLE (mode demand)
 *  
 */
 
const int STATE_IDLE = 0;
const int STATE_GRINDING = 1;
const int STATE_DONE = 2;

const int MODE_TIMER = 0;
const int MODE_DEMAND = 1;

int state = STATE_IDLE;
int mode = MODE_TIMER;

// these constants won't change:

const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
const int NUM_ADC_STATES = 1024;
const int COOL_DOWN = 3000; //number of seconds to cool down after grinding
const int ROT_ADJ_THRESH = 100; //0.1 of a second
const int GRIND_BUTTON = 6;
const int EVENT_BUTTON = 5; 
#ifdef pot
  const int POTI_PIN = 1;    // select the input pin for the potentiometer analogue pin 0
#endif
#ifdef rotary_encoder
  //swap pins on board to get it turning right way!
  const int CLK = A0;
  const int DT = A1;
  #define ENC_PORT PINC
#endif
const int STATUS_LED_PIN = 4; // pin9 is a PWM pin and allows for analogWrite.
//switching phase and neutral for safety
const int RELAY_PIN_L = 12;
const int DEBOUNCE_DELAY = 50;

// Variables will change:
unsigned long grind_start = 0;
unsigned long grind_time = 0;
unsigned long grind_debounce_time = 0;
unsigned long mode_debounce_time = 0;
unsigned long adjust_time_start = 0;
int status_led_brightness = 0;
int fade_rate = 10;
bool timer_starts;

int val = 0;
char buf[16];
int grind_time_preset = MIN_GRIND_TIME;

int sensorValue = 0;  // variable to store the value coming from the potentiaometer
int sensorValueNew = 0;

void setup() {
  //read preset storage
  EEPROM_readAnything(0, configuration);
  if (configuration.preset == -1) {
        configuration.preset = 9000;
  }
  EEPROM_writeAnything(0, configuration);
    
  // initialize the button pin as an input
  pinMode(GRIND_BUTTON, INPUT);
  digitalWrite(GRIND_BUTTON, LOW);
  // initialize the button LED as an output
  pinMode(STATUS_LED_PIN, OUTPUT);
  // initialize the RELAY as an output
//  pinMode(RELAY_PIN_N, OUTPUT);
//  digitalWrite(RELAY_PIN_N, LOW);
  pinMode(RELAY_PIN_L, OUTPUT);
  digitalWrite(RELAY_PIN_L, LOW);  
  //initialize event button
  pinMode(EVENT_BUTTON, INPUT);
  digitalWrite(EVENT_BUTTON, LOW); 
  // initialize serial communication:
  Serial.begin(115200);  
  // initialize lcd
  lcd.begin(16,2);
  lcd.backlight(); // finish with backlight on  
  lcd.clear();
  #ifdef rotary_encoder
    pinMode(DT, INPUT_PULLUP);
    pinMode(SLK, INPUT_PULLUP);
  #endif  
}

void mode_change() {
  if (mode == MODE_TIMER) {
    mode = MODE_DEMAND;
    delay(500);
  }
  else if (mode == MODE_DEMAND) {
    mode = MODE_TIMER;
    delay(500);
  }
}

long int new_val;

#ifdef roatry_encoder
 /* returns change in encoder state (-1,0,1) */
  int8_t read_encoder()
  {
    static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t old_AB = 0;
    /**/
    old_AB <<= 2;                   //remember previous state
    old_AB |= ( ENC_PORT & 0x03 );  //add current state
    return ( enc_states[( old_AB & 0x0f )]);
  }
#endif

void proc_idle() {
  #ifdef roatry_encoder
    //see https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
    static uint8_t counter = 0;      //this variable will be changed by encoder input
    int8_t tmpdata;
    /**/
    tmpdata = read_encoder();
    if( tmpdata ) { //if non-zero value returned
      #ifdef debug
        Serial.print("Counter value: ");
        Serial.println(counter, DEC);
      #endif
      counter += tmpdata; 
      grind_time_preset = (grind_time_preset + (tmpdata * 20)); //increment or decrement present value in multiples of 80ms
      if (grind_time_preset > MAX_GRIND_TIME) { //don't let it be more than max
        grind_time_preset = MAX_GRIND_TIME;
      }else if (grind_time_preset < MIN_GRIND_TIME) { //don't let it be less than min
        grind_time_preset = MIN_GRIND_TIME;
      }
      //update display every 100ms
      if (adjust_time_start == 0) {
        adjust_time_start = millis();
      }
      if (millis() - adjust_time_start > ROT_ADJ_THRESH) {
        update_display();
        configuration.preset = grind_time_preset;
        EEPROM_writeAnything(0, configuration);
        adjust_time_start = 0;
      }

    }    
  #endif
  #ifdef pot
    val = analogRead(POTI_PIN);
    grind_time_preset =  int(float(val)/(NUM_ADC_STATES-1) * (MAX_GRIND_TIME-MIN_GRIND_TIME)) + MIN_GRIND_TIME;
      if (abs(val-new_val) > 20 ) {
        update_display();
        new_val = val;
      }
  #endif

  //interupts not working for mode so read value of button
  if (digitalRead(EVENT_BUTTON) == HIGH) { 
    //do debounce stuff
    if (mode_debounce_time == 0){
      mode_debounce_time = millis();
    }
    if (millis() - mode_debounce_time > DEBOUNCE_DELAY) {
      //over debounce threshold so change mode
      mode_debounce_time = 0;
      mode_change();
      update_display();
    }
  }
  if (mode == MODE_TIMER) {
    if (digitalRead(GRIND_BUTTON) == HIGH) { 
      //do debounce stuff
      if (grind_debounce_time == 0){
        grind_debounce_time = millis();
      }
      if (millis() - grind_debounce_time > DEBOUNCE_DELAY) {
        //over debounce threshold so change state
        grind_start = 0;
        grind_time = 0;
        timer_starts = true;
        state = STATE_GRINDING;
        update_display();
        grind_debounce_time = 0;
      }
    }
  }
  //if not in timer mode need to grind on button push
  if (mode == MODE_DEMAND) {
    if (digitalRead(GRIND_BUTTON) == HIGH) { 
      //do debounce stuff
      if (grind_debounce_time == 0){
        grind_debounce_time = millis();
      }
      if (millis() - grind_debounce_time > DEBOUNCE_DELAY) {
        //over debounce threshold so change state
        state = STATE_GRINDING;
        update_display();
        grind_debounce_time = 0;
      }
    }
  }
}

void proc_grinding(){
  update_display();
  if (mode == MODE_TIMER) {
    //set flag for timer start
    if (timer_starts) {
      grind_start = millis();
      timer_starts = false;
    }
    grind_time = millis() - grind_start;   //grinding ends if grind time reached or event button is pressed to cancel
    if (grind_time > grind_time_preset){
      lcd.clear();
      update_display();
      #ifdef debug
        Serial.println("dropped out of grinding due to time reset trigger");
      #endif
      state = STATE_DONE;
    }
    //handle a cancellation with push of event button
    if (digitalRead(EVENT_BUTTON) == HIGH) { 
    //do debounce stuff
      if (mode_debounce_time == 0){
        mode_debounce_time = millis();
      }
      if (millis() - mode_debounce_time > DEBOUNCE_DELAY) {
        //over debounce threshold so stop grinding
        mode_debounce_time = 0;
        #ifdef debug
          Serial.println("Killed by event button");
        #endif
        state = STATE_DONE;
      }
    }
  }
  if (mode == MODE_DEMAND) {   //grind on demand so while button is pushed we will grind
    if (digitalRead(GRIND_BUTTON) == LOW){
      state = STATE_IDLE;
      lcd.clear();
      update_display();
    }
  }
}

void proc_done(){
  update_display();
  if (digitalRead(GRIND_BUTTON) == LOW) {
     manage_outputs();
     if (mode == MODE_TIMER) {
       delay(COOL_DOWN);
       state = STATE_IDLE;
     }
     if (mode == MODE_DEMAND) {
       state = STATE_IDLE;
     }
     lcd.clear();
     update_display();
  }
}

void manage_outputs(){
  if (state == STATE_GRINDING){
    digitalWrite(RELAY_PIN_L, HIGH);
//    digitalWrite(RELAY_PIN_N, HIGH);
//    analogWrite(STATUS_LED_PIN, status_led_brightness);
//    status_led_brightness += fade_rate;    
//    if (status_led_brightness <= 0 || status_led_brightness >= 255) {
//      fade_rate = -fade_rate;
//    }
    //delay(1); // delay to see fade
  }else{
    digitalWrite(RELAY_PIN_L, LOW);
//    digitalWrite(RELAY_PIN_N, LOW);
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
    Serial.print("    Grind time is: ");
    Serial.print(grind_time);
    Serial.print("    Grind start is: ");
    Serial.print(grind_start);
    Serial.print("     Mode is: ");
    Serial.println(mode);
   #endif
}









