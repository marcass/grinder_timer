/* Rotary encoder read example */
#define ENC_PORT PINC
const int CLK = A0;
const int DT = A1; 
const int MIN_GRIND_TIME = 5000;
const int MAX_GRIND_TIME = 15000;
long grind_time_preset = MIN_GRIND_TIME;
int prev_counter;  
void setup()
{
  /* Setup encoder pins as inputs */
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);
  digitalWrite(A1, HIGH);
  Serial.begin (115200);
  Serial.println("Start");
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
//    
    Serial.print("  Grind time = ");
    Serial.println(grind_time_preset);
    counter += tmpdata;
    x = counter % 4;
    if (x == 0) {
      grind_time_preset = (grind_time_preset + (tmpdata * 50));
      if (grind_time_preset > MAX_GRIND_TIME) {
        grind_time_preset = MAX_GRIND_TIME;
      }else if (grind_time_preset < MIN_GRIND_TIME) {
        grind_time_preset = MIN_GRIND_TIME;
      }
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
