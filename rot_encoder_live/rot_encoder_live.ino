//Runs a mazzer superjolly grinder with timer


//#define debug

#include <Wire.h>
//https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//preset storage (from reboot for rotary encoder value)
#include <EEPROM.h>
#include "/home/mw/git/grinder/EEPROMAnything.h"
struct config_t
{
   int preset;
} configuration;

//******************** setup encoder*************
#define ENC_PORT PINC
const int CLK = A0;
const int DT = A1; 

//************************ presets ***************
const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
const int STATE_IDLE = 0;
const int STATE_GRINDING = 1;
const int STATE_DONE = 2;
const int MODE_TIMER = 0;
const int MODE_DEMAND = 1;
const int COOL_DOWN = 3000; //number of seconds to cool down after grinding
const long EEPROM_ADJ_THRESH = 600000; //10min
const int DEBOUNCE_DELAY = 50;
//pins
const int GRIND_BUTTON = 6;
const int EVENT_BUTTON = 5; 
const int STATUS_LED_PIN = 9; // pin9 is a PWM pin and allows for analogWrite.
const int RELAY_PIN_L = 12;

//********************* variables ******************
long grind_time_preset;
long new_preset;
unsigned long grind_start = 0;
unsigned long grind_time = 0;
unsigned long adjust_time_start = 0;
char buf[16];
int state = STATE_IDLE;
int mode = MODE_TIMER;
bool newPreset = false;
bool timer_starts;
unsigned long mode_debounce_time = 0;
unsigned long grind_debounce_time = 0;
int status_led_brightness = 0;
int fade_rate = 10;


void setup() {
  // initialize the button pin as an input
  pinMode(GRIND_BUTTON, INPUT);
  digitalWrite(GRIND_BUTTON, LOW);
  // initialize the button LED as an output
  pinMode(STATUS_LED_PIN, OUTPUT);
  // initialize the RELAY as an output
  pinMode(RELAY_PIN_L, OUTPUT);
  digitalWrite(RELAY_PIN_L, LOW);  
  //initialize event button
  pinMode(EVENT_BUTTON, INPUT);
  digitalWrite(EVENT_BUTTON, LOW); 
  /* Setup encoder pins as inputs */
  pinMode(CLK, INPUT);
  digitalWrite(CLK, HIGH);
  pinMode(DT, INPUT);
  digitalWrite(DT, HIGH);
  Serial.begin (115200);
  Serial.println("Start");
  //read preset storage
  EEPROM_readAnything(0, configuration);
  if (configuration.preset == -1) {
    grind_time_preset = 6000;
  }else {
    grind_time_preset = configuration.preset;
  }
  EEPROM_writeAnything(0, configuration);
//  Serial.print("Grind time preset = ");
//  Serial.println(grind_time_preset);
  // initialize lcd
  lcd.begin(16,2);
  lcd.backlight(); // finish with backlight on  
  lcd.clear();
  update_display();
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


void proc_idle() {
 static uint8_t counter = 0;   //this variable will be changed by encoder input
 int8_t tmpdata;
 int x;
 /**/
  tmpdata = read_encoder();
  if( tmpdata ) {
//    Serial.print("Counter value: ");
//    Serial.print(counter, DEC);
//    Serial.print("tmpdata: ");
//    Serial.print(tmpdata);   
//    Serial.print("  Grind time = ");
//    Serial.println(grind_time_preset);
    counter += tmpdata;
    x = counter % 4;
    if (x == 0) {
      new_preset = (grind_time_preset + (tmpdata * 50));
      if (new_preset > MAX_GRIND_TIME) {
        grind_time_preset = MAX_GRIND_TIME;
      }else if (new_preset < MIN_GRIND_TIME) {
        grind_time_preset = MIN_GRIND_TIME;
      }else {
        grind_time_preset = new_preset;
      }
      update_display();
      newPreset = true;
    }
//  Serial.print("Counter value: ");
//  Serial.print(counter, DEC);
//  Serial.print("  Grind time = ");
//  Serial.println(grind_time_preset);
  }
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
    analogWrite(STATUS_LED_PIN, status_led_brightness);
    status_led_brightness += fade_rate;    
    if (status_led_brightness <= 0 || status_led_brightness >= 255) {
      fade_rate = -fade_rate;
    }
    //delay(1); // delay to see fade
  }else{
    digitalWrite(RELAY_PIN_L, LOW);
//    digitalWrite(RELAY_PIN_N, LOW);
    digitalWrite(STATUS_LED_PIN, HIGH);
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
  if (newPreset) { //start timer for EEPROM write
    Serial.println("new preset so waiting");
    if (adjust_time_start == 0 ) {
      adjust_time_start = millis();
    }
    if(millis() - adjust_time_start > EEPROM_ADJ_THRESH) {
      configuration.preset = grind_time_preset;
      EEPROM_writeAnything(0, configuration);
      newPreset = false;
      adjust_time_start = 0;
      Serial.print("wrote data  ");
      Serial.println(configuration.preset);
    }
  }
}
 
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
