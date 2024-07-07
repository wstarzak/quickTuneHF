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

#define MAX_SWR 11.3
#define MAX_TUNING_SWR 15.0

#define L_COUNT 7
#define C_COUNT 8

#define DEBUG true

struct Point
{
  int ind;
  int cap;
};

Point grid_array[256];
Vector<Point> grid_vector(grid_array);

// The registers needs to be cleared singe we have 8 output shift register
boolean registers_l[8];
boolean registers_c[8];

int cap_value;
int ind_value;

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
    registers_l[i] = LOW;
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
  int min_swr_ind_val = ind_value;

  for (uint16_t i = ind_step; i < max_ind_val; i += ind_step)
  {
    set_ind_value(i);
    writeRegistersL();
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    float current_swr = getSWR();
    if(DEBUG) {
      char s [128];
      sprintf (s, "[C_IND]: SWR: %4.2f, IT: %2u, L: %3d, C: %3d", current_swr, i, ind_value, (-1+(2*c_side_trx))*cap_value);
      Serial.println(s);
    }
    if(current_swr == 0.0) { break; } // TX stopped while tunnning
    if(current_swr < min_swr) {
      min_swr = current_swr;
      min_swr_ind_val = ind_value;
    }
    if((current_swr - min_swr) > 8.0) { break; } // We do not want really tune if the swr increase instead of decrease...
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
    if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({ind_value, (-1+(2*c_side_trx))*cap_value}); }
    float current_swr = getSWR();
    if(DEBUG) {
      char s [128];
      sprintf (s, "[C_CAP]: SWR: %4.2f, IT: %2u, L: %3d, C: %3d", current_swr, i, ind_value, (-1+(2*c_side_trx))*cap_value);
      Serial.println(s);
    }
    if(current_swr == 0.0) { break; } // TX stopped while tunnning
    if(current_swr < min_swr) {
      min_swr = current_swr;
      min_swr_cap_val = cap_value;
    }
    if((current_swr - min_swr) > 4.0) { break; } // We do not want really tune if the swr increase instead of decrease...
  }
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
}

void fine_tuning(uint16_t fine_step = 1) {
  int init_int_val = ind_value;
  int init_cap_val = cap_value;

  float min_swr = 99.9;
  int min_swr_cap_val = cap_value;
  int min_swr_ind_val = ind_value;
  for(int8_t ind = -fine_step; ind < 2; ind+=fine_step) {
    for(int8_t cap = -fine_step; cap < 2; cap+=fine_step) {
      bool in_grid = false;
      if(((init_int_val + ind) > (pow(2, L_COUNT) - 1)) || ((init_int_val + ind) < 0)) { continue; }
      if(((init_cap_val + cap) > (pow(2, C_COUNT) - 1)) || ((init_cap_val + cap) < 0)) { continue; }
      for (size_t i = 0; i < grid_vector.max_size(); i++)
      {
        if ((grid_vector.at(i).ind == (init_int_val + ind)) && (grid_vector.at(i).cap == ((-1+(2*c_side_trx))*(init_cap_val + cap))))
        {
          in_grid = true;
          break;
        }
        else
        {
          if (grid_vector.max_size() > grid_vector.size()) { grid_vector.push_back({(init_int_val + ind), (-1+(2*c_side_trx))*(init_cap_val + cap)}); }
        }
      }
      if(in_grid) { continue; }
      set_ind_value(init_int_val + ind);
      writeRegistersL();
      set_cap_value(init_cap_val + cap);
      writeRegistersC();
      float current_swr = getSWR();
      if(DEBUG) {
        char s [128];
        sprintf (s, "[FINE]: SWR: %4.2f, L: %3d, C: %3d", current_swr, ind_value, (-1+(2*c_side_trx))*cap_value);
        Serial.println(s);
      }
      if(current_swr < MAX_SWR) { tuned = true; return; }
      if(current_swr == 0.0) { break; } // TX stopped while tunnning
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
  for(int i=0; i<10; i++) {
    while(!FreqCount.available()) { continue; }
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
    float swr = getSWR();
    if(DEBUG) {
      char s [128];
      sprintf (s, "[FRQ]: %4.2fMHz SWR: %2.2f", frequency, swr);
      Serial.println(s);
    }
    if(swr > MAX_SWR) {
      tuned = false;
      reset_LC();
      coarse_ind();
      coarse_cap();
      Serial.println("Fine tune...");
      for(int i = 3; i > 0; i--) {
        for(int y = 0; y < 16; y++) {
          fine_tuning(i);
          if(tuned) { break; }
        }
        if(tuned) { break; }
      }
      delay(4000);
      if(getSWR() > MAX_SWR) {
        c_side_ant = LOW;
        c_side_trx = HIGH;
        writeRegistersC();
        coarse_cap();
      }
      delay(3000);
    }
  }
}