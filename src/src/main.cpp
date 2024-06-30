#include <FreqCount.h>
#include <Arduino.h>
#include <math.h>
#include <Vector.h>
#include "main.h"
#include "MegunoLink.h"

#define SER_PIN_L 17   // pin 14 on the 75HC595
#define RCLK_PIN_L 16  // pin 12 on the 75HC595
#define SRCLK_PIN_L 15 // pin 11 on the 75HC595
#define TLC_PIN_L 13

#define SER_PIN_C 11  // pin 14 on the 75HC595
#define RCLK_PIN_C 10 // pin 12 on the 75HC595
#define SRCLK_PIN_C 9 // pin 11 on the 75HC595
#define TLC_PIN_C 12

#define C_IN_PIN 7
#define C_OUT_PIN 8

#define SWR_REF_PIN A7
#define SWR_FWD_PIN A0

#define MAX_SWR 2.3
#define MAX_TUNING_SWR 15.0

#define DELAY_TIME 50

#define L_COUNT 7
#define C_COUNT 8

//#define MAX_L_VALUE (pow(2, L_COUNT))
//#define MAX_C_VALUE (pow(2, C_COUNT))

#define DEBUG_OUTPUT true
//#define MEGGUNO_ENABLED false

#ifdef MEGGUNO_ENABLED
  Table tune_params;
  XYPlot coarse_tune;
#endif

int MAX_L_VALUE = pow(2, L_COUNT);
int MAX_C_VALUE = pow(2, C_COUNT);

struct Point
{
  int ind;
  int cap;
};

Point grid_array[180];
Vector<Point> grid_vector(grid_array);

// we want to make sure that we are working on amateur bands
bool frequency_locked = false;

boolean registers_l[8];
boolean registers_c[8];

int cap_value;
int ind_value;

bool c_side_ant = HIGH;
bool c_side_trx = HIGH;

float swr_correction = 0;

float frequency = 0;

float tuned_swr;
float tuned_frequency;
bool tuned = false;

float fine_min_swr;
int fine_min_cap;
int fine_min_ind;

void lock_frequency()
{
  if (
      (frequency > 1.75 && frequency < 2.1) ||
      (frequency > 3.45 && frequency < 4.0) ||
      (frequency > 5.25 && frequency < 5.4) ||
      (frequency > 6.90 && frequency < 7.3) ||
      (frequency > 10.0 && frequency < 10.3) ||
      (frequency > 13.9 && frequency < 14.45) ||
      (frequency > 18.0 && frequency < 18.2) ||
      (frequency > 20.9 && frequency < 21.6) ||
      (frequency > 24.5 && frequency < 25.0) ||
      (frequency > 27.0 && frequency < 30.0))
  {
    frequency_locked = true;
  }
  swr_correction = 0.0;
  if (frequency > 1.75 && frequency < 2.1)
  {
    swr_correction = 28.0;
  }
  if (frequency > 3.45 && frequency < 4.0)
  {
    swr_correction = 25.0;
  }
  if (frequency > 5.25 && frequency < 5.4)
  {
    swr_correction = 20.0;
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(SER_PIN_L, OUTPUT);
  pinMode(RCLK_PIN_L, OUTPUT);
  pinMode(SRCLK_PIN_L, OUTPUT);
  pinMode(TLC_PIN_L, OUTPUT);

  pinMode(SER_PIN_C, OUTPUT);
  pinMode(RCLK_PIN_C, OUTPUT);
  pinMode(SRCLK_PIN_C, OUTPUT);
  pinMode(TLC_PIN_C, OUTPUT);

  pinMode(C_IN_PIN, OUTPUT);
  pinMode(C_OUT_PIN, OUTPUT);

  pinMode(SWR_REF_PIN, INPUT);
  pinMode(SWR_FWD_PIN, INPUT);

  #ifdef MEGGUNO_ENABLED
    coarse_tune.SetTitle("Coarse tune");
    coarse_tune.SetXlabel("IND");
    coarse_tune.SetYlabel("CAP");
    //coarse_tune.SetSeriesProperties("ADCValue", Plot::Magenta, Plot::Solid, 2, Plot::Square);
  #endif

  FreqCount.begin(100);

  // run bypass
  clearRegistersC();
  clearRegistersL();
  set_ind_value(0);
  writeRegistersL();
  set_cap_value(0);
  writeRegistersC();
  delay(DELAY_TIME);
}

// set all register pins to LOW
void clearRegistersL()
{
  for (int i = 7; i >= 0; i--)
  {
    registers_l[i] = LOW;
  }
  cap_value = 0;
  ind_value = 0;
}

// set all register pins to LOW
void clearRegistersC()
{
  for (int i = 8 - 1; i >= 0; i--)
  {
    registers_c[i] = LOW;
  }
  cap_value = 0;
  ind_value = 0;
}

// Set and display registers
// Only call AFTER all values are set how you would like (slow otherwise)
void writeRegistersC()
{
  digitalWrite(RCLK_PIN_C, LOW);
  //if(DEBUG_OUTPUT) Serial.print(" CP: ");
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(SRCLK_PIN_C, LOW);
    int val = registers_c[i];
    //if(DEBUG_OUTPUT) Serial.print(val);
    digitalWrite(SER_PIN_C, val);
    digitalWrite(SRCLK_PIN_C, HIGH);
  }
  //if(DEBUG_OUTPUT) Serial.print(" ");
  digitalWrite(RCLK_PIN_C, HIGH);
  digitalWrite(TLC_PIN_C, HIGH);
  digitalWrite(TLC_PIN_C, LOW);

  digitalWrite(C_IN_PIN, c_side_trx);
  digitalWrite(C_OUT_PIN, c_side_ant);
}

// Set and display registers
// Only call AFTER all values are set how you would like (slow otherwise)
void writeRegistersL()
{
  digitalWrite(RCLK_PIN_L, LOW);
  //if(DEBUG_OUTPUT) Serial.print(" L:");
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(SRCLK_PIN_L, LOW);
    int val = registers_l[i];
    //if(DEBUG_OUTPUT) Serial.print(val);
    digitalWrite(SER_PIN_L, val);
    digitalWrite(SRCLK_PIN_L, HIGH);
  }
  //if(DEBUG_OUTPUT) Serial.print(" ");
  digitalWrite(RCLK_PIN_L, HIGH);
  digitalWrite(TLC_PIN_L, HIGH);
  digitalWrite(TLC_PIN_L, LOW);

  digitalWrite(C_IN_PIN, c_side_trx);
  digitalWrite(C_OUT_PIN, c_side_ant);
  delay(DELAY_TIME);
}

void set_cap_value(int value)
{
  for (int i = 0; i < 8; i++)
    registers_c[i] = bitRead(value, i);
  cap_value = value;
}

void set_ind_value(int value)
{
  for (int i = 0; i < 8; i++)
    registers_l[i] = !bitRead(value, i);
  ind_value = value;
}

// set an individual pin HIGH or LOW
void setRegisterPinL(int index, int value)
{
  registers_l[index] = value;
}

void setRegisterPinC(int index, int value)
{
  registers_c[index] = value;
}

float getSWR()
{
  delay(DELAY_TIME);
  float ref = 1, fwd = 1;
  for (unsigned short i = 0; i < 10; i++)
  {
    ref += (float)analogRead(SWR_REF_PIN);
    fwd += (float)analogRead(SWR_FWD_PIN);
    delay(1);
  }
  if (ref < (10.0 * 10.0) && fwd < (10.0 * 10.0))
  { // todo: check if this is a good value
    return 0.0;
  }
  return (1.0 + (ref / fwd)) / (1.0 - (ref / fwd));
}

bool coarse_ind(int ind_steps = L_COUNT)
{
  Serial.println("Inductor coarse tunung...");
  set_ind_value(0);
  writeRegistersL();
  if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  float minimum_coarse_swr = getSWR();
  int minimum_coarse_ind_value = ind_value;
  float current_coarse_swr = minimum_coarse_swr;
  set_ind_value(1);
  writeRegistersL();

  if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  for(int i = 0; i < (ind_steps - 1); i++) {
    current_coarse_swr = getSWR();
    Serial.print("IND coarse: ");
    Serial.println(current_coarse_swr);
    if ((current_coarse_swr < minimum_coarse_swr) && ((minimum_coarse_swr - current_coarse_swr) > 0.5)) {
      minimum_coarse_swr = current_coarse_swr;   
      minimum_coarse_ind_value = ind_value;
    }
    set_ind_value((ind_value << 1));
    writeRegistersL();

    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  }
  set_ind_value(minimum_coarse_ind_value); // Lets go back with minimum if last one was higher
  writeRegistersL();

  // Lets do 2 iterations to get maybe btter SWR?
  if((ind_value % 2) == 1) return true;
  Serial.print("ind_val NOT 1\nSTEP: ");
  int step = ind_value/2;
  bool increased = false;
  // In case that no inductor is attached
  if(ind_value == 0) {
    step = 4;
    increased = true;
  }
  Serial.println(step);
  for(int i = 0; i < 3; i++) {
    set_ind_value(ind_value + step);
    writeRegistersL();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    current_coarse_swr = getSWR();
    Serial.print("IND fine inc: ");
    Serial.println(current_coarse_swr);
    if (current_coarse_swr < minimum_coarse_swr) {
      minimum_coarse_swr = current_coarse_swr; 
      minimum_coarse_ind_value = ind_value;
      if((ind_value % 2) == 1) return true;
      step /= 2;
      Serial.print("ind_val NOT 1\nSTEP: ");
      Serial.println(step);
      increased = true;
    }
  }
  set_ind_value(minimum_coarse_ind_value);
  writeRegistersL();

  if(increased) return true;

  for(int i = 0; i < 3; i++) {
    set_ind_value(ind_value - step);
    writeRegistersL();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    current_coarse_swr = getSWR();
    Serial.print("IND fine dec: ");
    Serial.println(current_coarse_swr);
    if (current_coarse_swr < minimum_coarse_swr) {
      minimum_coarse_swr = current_coarse_swr;   
      minimum_coarse_ind_value = ind_value;
      if((ind_value % 2) == 1) return true;
      step /= 2;
      Serial.print("ind_val NOT 1\nSTEP: ");
      Serial.println(step);
    }
  }
  set_ind_value(minimum_coarse_ind_value);
  writeRegistersL();
  return true;
}

bool coarse_cap(int cap_steps = C_COUNT)
{
  Serial.println("Capacitor coarse tunung...");
  set_cap_value(0);
  writeRegistersC();

  if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  float minimum_coarse_swr = getSWR();
  int minimum_coarse_cap_value = cap_value;
  float current_coarse_swr = minimum_coarse_swr;

  set_cap_value(1);
  writeRegistersC();

  if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  for(int i = 0; i < (cap_steps - 1); i++) {
    current_coarse_swr = getSWR();
    Serial.print("CAP coarse: ");
    Serial.println(current_coarse_swr);
    if (current_coarse_swr < minimum_coarse_swr) {
      minimum_coarse_swr = current_coarse_swr;   
      minimum_coarse_cap_value = cap_value;
    }
    if((current_coarse_swr - minimum_coarse_swr) > 2.0) break;
    set_cap_value((cap_value << 1));
    writeRegistersC();

    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
  }
  set_cap_value(minimum_coarse_cap_value); // Lets go back with minimum if last one was higher
  writeRegistersC();

  // Lets do 2 iterations to get maybe btter SWR?
  if((cap_value % 4) == 1) return true;
  int step = cap_value/4;
  bool increased = false;
  Serial.print("cap_val NOT 1\nSTEP: ");
  if(cap_value == 0) {
    step = 4;
    increased = true;
  }
  Serial.println(step);
  
  for(int i = 0; i < 3; i++) {
    set_cap_value(cap_value + step);
    writeRegistersC();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    current_coarse_swr = getSWR();
    Serial.print("\nCAP fine increase: ");
    Serial.println(current_coarse_swr);
    if (current_coarse_swr < minimum_coarse_swr) {
      minimum_coarse_swr = current_coarse_swr;   
      minimum_coarse_cap_value = cap_value;
      if((cap_value % 2) == 1) return true;
      step /= 2;
      Serial.print("cap_val NOT 1\nSTEP: ");
      Serial.println(step);
      increased = true;
    }
  }
  set_cap_value(minimum_coarse_cap_value);
  writeRegistersC();
  if(increased) return true;

  if((cap_value % 4) == 1) return true;
  step = cap_value/4;
  Serial.print("cap_val NOT 1\nSTEP: ");
  Serial.println(step);

  for(int i = 0; i < 3; i++) {
    set_cap_value(cap_value - step);
    writeRegistersC();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    current_coarse_swr = getSWR();
    Serial.print("\nCAP fine decrease: ");
    Serial.println(current_coarse_swr);
    if (current_coarse_swr < minimum_coarse_swr) {
      minimum_coarse_swr = current_coarse_swr;   
      minimum_coarse_cap_value = cap_value;
      if((cap_value % 2) == 1) return true;
      step /= 2;
      Serial.print("cap_val NOT 1\nSTEP: ");
      Serial.println(step);
    }
  }
  set_cap_value(minimum_coarse_cap_value);
  writeRegistersC();
  return true;
}

void fine_tuning(int step = 2, int iteration = 0)
{
  float current_swr;
  int ind_min = ind_value - step;
  int cap_min = cap_value - step;
  int ind_max = ind_value + step;
  int cap_max = cap_value + step;
  int second_jump = 1; // we do not want to jump every 1 value on low freq
  if (frequency < 16) { second_jump = 2; }
  if (frequency < 3) { second_jump = 3; }
  
  for(int ind = ind_min; ind <= ind_max; ind += 2*step) {
    for(int cap = cap_min; cap <= cap_max; cap += second_jump) {
      if (cap > MAX_C_VALUE || cap < 0) { continue; }
      if (ind > MAX_L_VALUE || ind < 0) { continue; }
      Point tmp_point;
      if(c_side_trx) {
        tmp_point = {ind, cap};
      } else {
        tmp_point = {ind, -cap};
      }
      bool in_grid = false;
      for (size_t i = 0; i < grid_vector.max_size(); i++)
      {
        if ((grid_vector.at(i).ind == tmp_point.ind) && (grid_vector.at(i).cap == tmp_point.cap))
        {
          in_grid = true;
          break;
        }
      }
      if(in_grid) { continue; }
      if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back(tmp_point); } else { break; }
      set_ind_value(ind);
      set_cap_value(cap);
      writeRegistersC();
      writeRegistersL();
      delay(DELAY_TIME);
      #ifdef MEGGUNO_ENABLED
      if(c_side_trx) {
        coarse_tune.SendData("FINE1", ind, cap);
      } else {
        coarse_tune.SendData("FINE1", ind, -cap);
      }
      #endif
      current_swr = getSWR();
      if (current_swr == 0.0) { return; }
      if (current_swr < 1.0) { continue; }
      if (current_swr < fine_min_swr)
      {
        fine_min_cap = cap;
        fine_min_ind = ind;
        fine_min_swr = current_swr;
        if(DEBUG_OUTPUT) {
          Serial.print("Fine minimum SWR: ");
          Serial.println(fine_min_swr);
        }
        if (fine_min_swr <= MAX_SWR)
        {
          return;
        }
      }
    }
  }
  for(int cap = cap_min; cap <= cap_max; cap += 2*step) {
    for(int ind = ind_min; ind <= ind_max; ind += second_jump) {
      if (cap > MAX_C_VALUE || cap < 0) { continue; }
      if (ind > MAX_L_VALUE || ind < 0) { continue; }
      Point tmp_point;
      if(c_side_trx) {
        tmp_point = {ind, cap};
      } else {
        tmp_point = {ind, -cap};
      }
      bool in_grid = false;
      for (size_t i = 0; i < grid_vector.max_size(); i++)
      {
        if ((grid_vector.at(i).ind == tmp_point.ind) && (grid_vector.at(i).cap == tmp_point.cap))
        {
          in_grid = true;
          break;
        }
      }
      if(in_grid) { continue; }
      if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back(tmp_point); } else { break; }
      set_ind_value(ind);
      set_cap_value(cap);
      writeRegistersC();
      writeRegistersL();
      delay(DELAY_TIME);
      #ifdef MEGGUNO_ENABLED
      if(c_side_trx) {
        coarse_tune.SendData("FINE2", ind, cap);
      } else {
        coarse_tune.SendData("FINE2", ind, -cap);
      }
      #endif
      current_swr = getSWR();
      if (current_swr == 0.0) { return; }
      if (current_swr < 1.0) { continue; }
      if (current_swr < fine_min_swr)
      {
        fine_min_cap = cap;
        fine_min_ind = ind;
        fine_min_swr = current_swr;
        if(DEBUG_OUTPUT) {
          Serial.print("Fine minimum SWR: ");
          Serial.println(fine_min_swr);
        }
        if (fine_min_swr <= MAX_SWR)
        {
          return;
        }
      }
    }
  }
  set_cap_value(fine_min_cap);
  set_ind_value(fine_min_ind);
  writeRegistersC();
  writeRegistersL();
  delay(DELAY_TIME);
  //if(iteration == 0) { return; } //5
  if(iteration == 0) {
    if(frequency < 15.0) { step = 3; }
    if(frequency < 3.0 ) { step = 6; }
    if(frequency > 15.0) { step = 2; }
  }
  if(iteration == 1) {
    if(frequency < 15.0) { step = 2; }
    if(frequency < 3.0 ) { step = 4; }
    if(frequency > 15.0) { step = 2; }
  }
  if(iteration == 2) {
    if(frequency < 15.0) { step = 1; }
    if(frequency < 3.0 ) { step = 2; }
    if(frequency > 15.0) { step = 1; }
  }
  if(iteration == 3) {
    if(frequency < 15.0) { step = 1; }
    if(frequency < 3.0 ) { step = 1; }
    if(frequency > 15.0) { step = 1; }
  }
  if(iteration == 4) { return; } //5
  fine_tuning(step, iteration+1);
  if(DEBUG_OUTPUT) {
    Serial.print("Lowest SWR for fine tuning: ");
    Serial.println(fine_min_swr);
  }
}

void reset_LC() {
  // Lets start from beginging
  c_side_ant = HIGH;
  c_side_trx = LOW;
  set_cap_value(0);
  set_ind_value(0);
  writeRegistersL();
  writeRegistersC();
  delay(DELAY_TIME);
}

void loop()
{
  //if (FreqCount.available())
  //{
  //  //if(DEBUG_OUTPUT) { Serial.print("FREQ: "); Serial.println(FreqCount.read()); Serial.print("SWR: "); Serial.println(getSWR());}
  //  #ifdef MEGGUNO_ENABLED
  //    tune_params.SendData(F("loop_frequency"), frequency);
  //  #endif
  //}
  if(getSWR() > MAX_SWR) {
    reset_LC();
    coarse_ind(L_COUNT);
    coarse_cap(C_COUNT);
    delay(2000);
    if(getSWR() > MAX_SWR) {
      Serial.println("Switch capacitor sides");
      c_side_ant = LOW;
      c_side_trx = HIGH;
      writeRegistersC();
      coarse_cap(C_COUNT);
    }
    delay(2000);
  }
  // lock_frequency();
  // float swr = getSWR();

  // #ifdef MEGGUNO_ENABLED
  //   if(swr > 0.0) tune_params.SendData(F("loop_swr"), swr);
  // #endif

  // if(swr > MAX_TUNING_SWR || swr < 0.0) {
  //   tuned = false; 
  //   reset_LC();
  // }

  // if(swr < MAX_SWR && swr >= 1.0) {
  //   tuned = true;
  // }

  // if(tuned && frequency_locked) {
  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("standby_swr"), swr);
  //     tune_params.SendData(F("standby_frequency"), frequency);
  //   #endif
  //   if (abs(frequency - tuned_frequency) > 0.15 && swr > MAX_SWR)
  //   {
  //     tuned = false; 
  //     reset_LC();
  //   }
  //   if(swr > MAX_SWR && (swr - tuned_swr) > 0.35) { 
  //     tuned = false; 
  //     reset_LC();
  //   }
  // }

  // if ((swr > MAX_SWR) && frequency_locked && !tuned && (swr < MAX_TUNING_SWR))
  // {
  //   tuned = false;
  //   tuned_swr = 999.9;
  //   tuned_frequency = frequency;
  //   reset_LC();

  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("frequency"), frequency);
  //   #endif

  //   MAX_L_VALUE = pow(2, L_COUNT);
  //   MAX_C_VALUE = pow(2, C_COUNT);
  //   if(frequency > 16.0) { 
  //     MAX_L_VALUE = pow(2, L_COUNT-2);
  //     MAX_C_VALUE = pow(2, C_COUNT-2);
  //   }
  //   if(frequency > 9.0 && frequency < 16.0) { 
  //     MAX_L_VALUE = pow(2, L_COUNT-1);
  //     MAX_C_VALUE = pow(2, C_COUNT-1);
  //   }

  //   if(DEBUG_OUTPUT) Serial.println("");

  //   if(frequency < 3.0) coarse_ind(L_COUNT); 
  //   if(frequency > 3.0 && frequency < 15.0) coarse_ind(L_COUNT-1);
  //   if(frequency > 16.0) coarse_ind(L_COUNT-2);
    
  //   swr = getSWR();
  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("ind_swr"), swr);
  //   #endif
    
  //   // CAP SIDE 1
  //   if(frequency < 3.0) coarse_cap(C_COUNT); 
  //   if(frequency > 3.0 && frequency < 15.0) coarse_cap(C_COUNT-2);
  //   if(frequency > 16.0) coarse_cap(C_COUNT-3);

  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("cap1_swr"), getSWR());
  //   #endif
  //   // CAP SIDE 2
  //   if(swr <= getSWR()) {
  //     c_side_ant = LOW;
  //     c_side_trx = HIGH;
  //     set_cap_value(0);
  //     writeRegistersL();
  //     writeRegistersC();
  //     if(frequency < 3.0) coarse_cap(C_COUNT);
  //     if(frequency > 3.0 && frequency < 15.0) coarse_cap(C_COUNT-2);
  //     if(frequency > 16.0) coarse_cap(C_COUNT-3);
  //   }
  //   fine_min_swr = getSWR();
  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("cap2_swr"), fine_min_swr);
  //   #endif
  //   fine_min_cap = cap_value;
  //   fine_min_ind = ind_value;
  //   if(frequency < 3.0) fine_tuning(7); 
  //   if(frequency > 3.0 && frequency < 15.0) fine_tuning(4);
  //   if(frequency > 16.0) fine_tuning(2);
  //   tuned = true;
  //   tuned_swr = getSWR();
    
  //   #ifdef MEGGUNO_ENABLED
  //     tune_params.SendData(F("tuned_swr"), tuned_swr);
  //   #endif
  //   tuned_frequency = frequency;
  //   delay(1000);
  // }

  // grid_vector.clear();
  // frequency = 0;
  // frequency_locked = false;
}