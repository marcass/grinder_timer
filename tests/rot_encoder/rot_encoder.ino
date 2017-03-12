/* Rotary encoder read example */
#define ENC_PORT PINC
const int CLK = A0;
const int DT = A1; 
const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
long grind_time_preset = MIN_GRIND_TIME;
long new_preset;
int prev_counter; 
char buf[16];
unsigned long previousMillis = 0;

#include <Wire.h>
//https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


void setup() {
  /* Setup encoder pins as inputs */
  pinMode(CLK, INPUT);
  digitalWrite(CLK, HIGH);
  pinMode(DT, INPUT);
  digitalWrite(DT, HIGH);
  Serial.begin (115200);
  Serial.println("Start");
  // initialize lcd
  lcd.begin(16,2);
  lcd.backlight(); // finish with backlight on  
  lcd.clear();
}


void update_display(){
  int bar_frac;
  //lcd.write("fuck");
  lcd.write("Timer  ");
  lcd.setCursor(4,1);
  sprintf(buf, "%5d ms", grind_time_preset);
  lcd.write(buf);
  
}


void loop()
{
 static uint8_t counter = 0;   //this variable will be changed by encoder input
 int8_t tmpdata;
 int x;
 /**/
  tmpdata = read_encoder();
  if( tmpdata ) {
    Serial.print("Counter value: ");
    Serial.print(counter, DEC);
    Serial.print("tmpdata: ");
    Serial.print(tmpdata);   
    Serial.print("  Grind time = ");
    Serial.println(grind_time_preset);
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
    }
  Serial.print("Counter value: ");
  Serial.print(counter, DEC);
  Serial.print("  Grind time = ");
  Serial.println(grind_time_preset);
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
