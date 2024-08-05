#include <FreqCount.h>
#include <Arduino.h>
#include <math.h>
#include <Vector.h>
#include "main.h"

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

#define MAX_SWR 1.3
#define MAX_TUNING_SWR 15.0

#define L_COUNT 7
#define C_COUNT 8

#define DEBUG true

#define TUNING_STEP_160M 5
#define TUNING_STEP_80M 4
#define TUNING_STEP_60M 3
#define TUNING_STEP_40M 3
#define TUNING_STEP_30M 2
#define TUNING_STEP_20M 2
#define TUNING_STEP_18M 1
#define TUNING_STEP_15M 1
#define TUNING_STEP_12M 1
#define TUNING_STEP_10M 1

struct Point
{
  uint16_t ind;
  int16_t cap;
};

Point grid_array[128];
Vector<Point> grid_vector(grid_array);

// The registers needs to be cleared singe we have 8 output shift register
boolean registers_l[8];
boolean registers_c[8];

int16_t cap_value;
uint16_t ind_value;

float frequency;
bool frequency_lock = false;

bool c_side_ant = HIGH;
bool c_side_trx = LOW;

bool tuned = false;

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

  FreqCount.begin(100);

  // Tuner starts with bypass mode
  clearRegistersC();
  clearRegistersL();
  writeRegistersC();
  writeRegistersL();
}

// set all register pins to LOW
void clearRegistersL()
{
  for (int i = 0; i < L_COUNT; i++)
  {
    registers_l[i] = HIGH;
  }
  ind_value = 0;
}

// set all register pins to LOW
void clearRegistersC()
{
  for (int i = 0; i < C_COUNT; i++)
  {
    registers_c[i] = LOW;
  }
  cap_value = 0;
}

void writeRegistersC()
{
  digitalWrite(RCLK_PIN_C, LOW);
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(SRCLK_PIN_C, LOW);
    digitalWrite(SER_PIN_C, registers_c[i]);
    digitalWrite(SRCLK_PIN_C, HIGH);
  }
  digitalWrite(RCLK_PIN_C, HIGH);
  digitalWrite(TLC_PIN_C, HIGH);
  digitalWrite(TLC_PIN_C, LOW);

  digitalWrite(C_IN_PIN, c_side_trx);
  digitalWrite(C_OUT_PIN, c_side_ant);
  delay(15); // safe value so the relay can set itself to desired state
}

void writeRegistersL()
{
  digitalWrite(RCLK_PIN_L, LOW);
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(SRCLK_PIN_L, LOW);
    digitalWrite(SER_PIN_L, registers_l[i]);
    digitalWrite(SRCLK_PIN_L, HIGH);
  }
  digitalWrite(RCLK_PIN_L, HIGH);
  digitalWrite(TLC_PIN_L, HIGH);
  digitalWrite(TLC_PIN_L, LOW);

  delay(15); // safe value so the relay can set itself to desired state
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
  float ref = 1, fwd = 1;

  // Average the reading (to be honest no idea why...)
  for (unsigned short i = 0; i < 3; i++)
  {
    ref += (float)analogRead(SWR_REF_PIN);
    fwd += (float)analogRead(SWR_FWD_PIN);
  }
  if (ref < (10.0 * 10.0) && fwd < (10.0 * 10.0))
  { // todo: check if this is a good value
    return 0.0;
  }
  return (1.0 + (ref / fwd)) / (1.0 - (ref / fwd));
}

void coarse_ind(uint16_t ind_step = 3) // x 3 should be a variable dependent on frequency
{
  uint16_t max_ind_val = pow(2, L_COUNT);
  
  float min_swr = 99.9;
  uint16_t min_swr_ind_val = ind_value;

  for (uint16_t i = ind_step; i < max_ind_val; i += ind_step)
  {
    set_ind_value(i);
    writeRegistersL();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (int16_t)(-1+(2*c_side_trx))*cap_value}); }
    float current_swr = getSWR();
    if(DEBUG) {
      char s [64];
      char str_swr[5] = {0};
      dtostrf(current_swr, 2, 2, str_swr);
      sprintf (s, "[C_IND]: SWR: %s, IT: %2u, L: %3d, C: %3d", str_swr, i, ind_value, (int16_t)(-1+(2*c_side_trx))*cap_value);
      Serial.println(s);
    }
    if(current_swr == 0.0) { tuned = false; return; } // TX stopped while tunnning
    if(current_swr < min_swr) {
      min_swr = current_swr;
      min_swr_ind_val = ind_value;
    }
    if((current_swr - min_swr) > 1.0) { break; } // We do not want really tune if the swr increase instead of decrease...
  }
  set_ind_value(min_swr_ind_val);
  writeRegistersL();
}

void coarse_cap(uint16_t cap_step = 3) // x 8 should be a variable dependent on frequency
{
  uint16_t max_cap_val = pow(2, C_COUNT);

  float min_swr = 99.9;
  int min_swr_cap_val = cap_value;

  for (uint16_t i = cap_step; i < max_cap_val; i += cap_step)
  {
    set_cap_value(i);
    writeRegistersC();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (int16_t)(-1+(2*c_side_trx))*cap_value}); }
    float current_swr = getSWR();
    if(DEBUG) {
      char s [64];
      char str_swr[5] = {0};
      dtostrf(current_swr, 2, 2, str_swr);
      sprintf (s, "[C_CAP]: SWR: %s, IT: %2u, L: %3d, C: %3d", str_swr, i, ind_value, (-1+(2*c_side_trx))*cap_value);
      Serial.println(s);
    }
    if(current_swr == 0.0) { tuned = false; return; } // TX stopped while tunnning
    if(current_swr < min_swr) {
      min_swr = current_swr;
      min_swr_cap_val = cap_value;
    }
    if((current_swr - min_swr) > 1.0) { break; } // We do not want really tune if the swr increase instead of decrease...
  }
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
}

void fine_tuning(uint16_t fine_step = 1) {
  uint16_t init_ind_val = ind_value;
  int16_t init_cap_val = cap_value;

  float min_swr = 99.9;
  int16_t min_swr_cap_val = cap_value;
  uint16_t min_swr_ind_val = ind_value;
  for(uint16_t ind = -fine_step; ind < 2; ind+=fine_step) {
    for(int16_t cap = -fine_step; cap < 2; cap+=fine_step) {
      bool in_grid = false;
      if(((init_ind_val + ind) > (pow(2, L_COUNT) - 1)) || ((init_ind_val + ind) < 0)) { continue; }
      if(((init_cap_val + cap) > (pow(2, C_COUNT) - 1)) || ((init_cap_val + cap) < 0)) { continue; }
      for (size_t i = 0; i < grid_vector.max_size(); i++)
      {
        if ((grid_vector.at(i).ind == (init_ind_val + ind)) && (grid_vector.at(i).cap == ((-1+(2*c_side_trx))*(init_cap_val + cap))))
        {
          in_grid = true;
          break;
        }
        else
        {
          if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({(init_ind_val + ind), (int16_t)(-1+(2*c_side_trx))*(init_cap_val + cap)}); }
        }
      }
      if(in_grid) { continue; }
      set_ind_value(init_ind_val + ind);
      writeRegistersL();
      set_cap_value(init_cap_val + cap);
      writeRegistersC();
      float current_swr = getSWR();
      if(current_swr == 0.0) { tuned = false; return; }
      if(DEBUG) {
        tuned = false;
        char s [64];
        char str_swr[5] = {0};
        dtostrf(current_swr, 2, 2, str_swr);
        sprintf (s, "[FINE]: SWR: %s, L: %3d, C: %3d", str_swr, ind_value, (-1+(2*c_side_trx))*cap_value);
        Serial.println(s);
      }
      if(current_swr < MAX_SWR) { tuned = true; return; }
      if(current_swr == 0.0) { tuned = false; break; } // TX stopped while tunnning
      if(current_swr < min_swr) {
        min_swr = current_swr;
        min_swr_cap_val = cap_value;
        min_swr_ind_val = ind_value;
      }
    }
  }
  set_ind_value(min_swr_ind_val);
  writeRegistersL();
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
}

void reset_LC() {
  c_side_ant = HIGH;
  c_side_trx = LOW;
  clearRegistersL();
  clearRegistersC();
  writeRegistersL();
  writeRegistersC();
}

void loop()
{
  float freq_lock[10];
  frequency_lock = false;
  uint16_t tuning_step = 1;

  for(int i = 0; i<10; i++) {
    while(!FreqCount.available()) { if(getSWR() == 0.0) break; else continue; }
    if(getSWR() == 0.0) {
      frequency_lock = false;
      break;
    }
    frequency = (((float)FreqCount.read() * 4096.0 / 100000.0));
    freq_lock[i] = frequency;
    if(i > 3) {
      if (freq_lock[i-1] == freq_lock[i] &&
          freq_lock[i-2] == freq_lock[i] &&
          freq_lock[i-3] == freq_lock[i]) 
        {
          if(frequency > 1.0 && frequency < 30.0) {
            frequency_lock = true;
          }
        } else {
          frequency_lock = false;
        }
        if(frequency_lock == true) { break; }
    }
  }

  if(frequency_lock) {
    switch ((int)floor(frequency))
    {
    case 1:
      tuning_step = TUNING_STEP_160M;
      break;
    case 3:
      tuning_step = TUNING_STEP_80M;
      break;
    case 5:
      tuning_step = TUNING_STEP_60M;
      break;
    case 7:
      tuning_step = TUNING_STEP_40M;
      break;
    case 10:
      tuning_step = TUNING_STEP_30M;
      break;
    case 14:
      tuning_step = TUNING_STEP_20M;
      break;
    case 18:
      tuning_step = TUNING_STEP_18M;
      break;
    case 21:
      tuning_step = TUNING_STEP_15M;
      break;
    case 24:
      tuning_step = TUNING_STEP_12M;
      break;
    case 30:
      tuning_step = TUNING_STEP_10M;
      break;
    default:
      break;
    }
    float swr = getSWR();
    float swr_old = swr;
    grid_vector.clear();
    if(DEBUG) {
      swr = getSWR();
      char s[128] = {0};
      char str_swr[7];
      char str_frq[8];
      dtostrf(frequency, 4, 2, str_frq);
      dtostrf(swr, 2, 2, str_swr);
      sprintf(s, "[FRQ]: %sMHz SWR: %s", str_frq, str_swr);
      Serial.println(s);
    }
    if((swr > MAX_SWR) && (swr < MAX_TUNING_SWR)) {
      tuned = false;
      reset_LC();
      coarse_ind(tuning_step);
      coarse_cap(tuning_step);
      swr = getSWR();
      if((swr >= swr_old) || (swr > MAX_SWR)) {
        c_side_ant = LOW;
        c_side_trx = HIGH;
        clearRegistersC();
        writeRegistersC();
        coarse_cap(tuning_step);
      }
      Serial.println("Fine tune...");
      for(int i = 3; i > 0; i--) {
        for(int y = 0; y < 3; y++) {
          fine_tuning(i); // one last pass to finetune
          if(tuned) { break; } 
        }
        if(tuned) { break; }
      }
      delay(2000);
    }
  }
}