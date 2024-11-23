#include "main.h"
#include "pinout.h"

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

struct FrqMemPoint
{
  float frq;
  uint16_t ind;
  int16_t cap;
};

Point grid_array[350];
Vector<Point> grid_vector(grid_array);

FrqMemPoint freq_array[32];
Vector<FrqMemPoint> freq_vector(freq_array);

// The registers needs to be cleared singe we have 8 output shift register
boolean registers_l[8];
boolean registers_c[8];

int16_t cap_value;
uint16_t ind_value;

float frequency;
bool frequency_lock = false;
SlidingWindow freq_lock(10);

bool c_side_ant = HIGH;
bool c_side_trx = LOW;

bool tuned = false;
float tuned_frequency = 999.9;

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

// set all register pins to HIGH
// We want to short all of the inductors
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
             // For HF115F operation time is around 15ms
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

float getSWR(uint8_t average = 1)
{
  float ref = 1, fwd = 1;

  for (uint8_t i = 0; i < average; i++)
  {
    ref += (float)analogRead(SWR_REF_PIN);
    fwd += (float)analogRead(SWR_FWD_PIN);
  }
  
  if (ref < 10.0 && fwd < 10.0)
  {
    return 0.0;
  }

  return (1.0 + (ref / fwd)) / (1.0 - (ref / fwd));
}

// EXIT CODES
// 0 - succesfully decreased SWR
// 1 - failed to decrease
// 2 - TX stopped
uint8_t coarse_ind(uint16_t ind_step)
{
  uint16_t max_ind_val = pow(2, L_COUNT);
  uint16_t min_swr_ind_val = ind_value;

  uint8_t exit_code = 1;

  float min_swr = 99.9;

  for (uint16_t i = min_swr_ind_val; i < max_ind_val; i += ind_step)
  {
    set_ind_value(i);
    writeRegistersL();

    if (grid_vector.max_size() > grid_vector.size())
    {
      grid_vector.push_back({ind_value, (int16_t)(-1 + (2 * c_side_trx)) * cap_value});
    }

    float current_swr = getSWR();
    if (current_swr == 0.0) // TX stopped while tunnning
    {
      tuned = false;
      return 2;
    }

    if (current_swr < min_swr) // Update minimum SWR & inductor values if the SWR starts decreasing 
    {
      min_swr = current_swr;
      min_swr_ind_val = ind_value;
      exit_code = 0;
    }

    if ((current_swr - min_swr) > 1.0) // We do not want really tune if the swr increase instead of decrease...
    {
      break;
    }
  }

  // At the end of tuning routine set inductor value with corresponded with minimum SWR
  set_ind_value(min_swr_ind_val);
  writeRegistersL();
  return exit_code;
}

// EXIT CODES
// 0 - succesfully decreased SWR
// 1 - failed to decrease
// 2 - TX stopped
uint8_t coarse_cap(uint16_t cap_step)
{
  uint16_t max_cap_val = pow(2, C_COUNT);
  int16_t min_swr_cap_val = cap_value;

  uint8_t exit_code = 1;

  float min_swr = 99.9;

  for (uint16_t i = min_swr_cap_val; i < max_cap_val; i += cap_step)
  {
    set_cap_value(i);
    writeRegistersC();
    if (grid_vector.max_size() > grid_vector.size())
    {
      grid_vector.push_back({ind_value, (int16_t)(-1 + (2 * c_side_trx)) * cap_value});
    }

    float current_swr = getSWR();
    if (current_swr == 0.0) // TX stopped while tunnning
    {
      tuned = false;
      return 2;
    }

    if (current_swr < min_swr) // Update minimum SWR & inductor values if the SWR starts decreasing 
    {
      min_swr = current_swr;
      min_swr_cap_val = cap_value;
      exit_code = 0;
    }

    if ((current_swr - min_swr) > 1.0) // We do not want really tune if the swr increase instead of decrease...
    {
      break;
    } 
  }

  // At the end of tuning routine set capacitor value with corresponded with minimum SWR
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
  return exit_code;
}

// EXIT CODES
// 0 - succesfully decreased SWR
// 1 - failed to decrease
// 2 - TX stopped
uint8_t fine_tuning(uint16_t fine_step = 1)
{
  uint16_t init_ind_val = ind_value;
  int16_t init_cap_val = cap_value;

  uint8_t exit_code = 1;

  float min_swr = 99.9;
  int16_t min_swr_cap_val = cap_value;
  uint16_t min_swr_ind_val = ind_value;
  
  // This checks all combinations around minimum value so we get best match
  for (uint16_t ind = -fine_step; ind < 2; ind += fine_step)
  {
    for (int16_t cap = -fine_step; cap < 2; cap += fine_step)
    {
      bool in_grid = false;

      // Lets pass those values which are too high or too low to set by the ATU
      if (((init_ind_val + ind) > (pow(2, L_COUNT) - 1)) || ((init_ind_val + ind) < 0))
      {
        continue;
      }
      if (((init_cap_val + cap) > (pow(2, C_COUNT) - 1)) || ((init_cap_val + cap) < 0))
      {
        continue;
      }

      for (size_t i = 0; i < grid_vector.max_size(); i++)
      {
        if ((grid_vector.at(i).ind == (init_ind_val + ind)) && (grid_vector.at(i).cap == ((-1 + (2 * c_side_trx)) * (init_cap_val + cap))))
        {
          in_grid = true;
          break;
        }
        else
        {
          if (grid_vector.max_size() > grid_vector.size())
          {
            grid_vector.push_back({(init_ind_val + ind), (int16_t)(-1 + (2 * c_side_trx)) * (init_cap_val + cap)});
          }
        }
      }

      if (in_grid)
      {
        continue;
      }
      set_ind_value(init_ind_val + ind);
      writeRegistersL();
      set_cap_value(init_cap_val + cap);
      writeRegistersC();

      float current_swr = getSWR();
      if (current_swr == 0.0)
      {
        tuned = false;
        return 2;
      }

      if (current_swr < MAX_SWR)
      {
        tuned = true;
        tuned_frequency = frequency;
        freq_vector.push_back({frequency, ind_value, (int16_t)(-1 + (2 * c_side_trx)) * (cap_value)});
        return 0;
      }

      if (current_swr < min_swr)
      {
        min_swr = current_swr;
        min_swr_cap_val = cap_value;
        min_swr_ind_val = ind_value;
        exit_code = 0;
      }
    }
  }
  set_ind_value(min_swr_ind_val);
  writeRegistersL();
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
  return(exit_code);
}

int set_tuning_step(float frequency)
{
  switch ((int)floor(frequency))
  {
  case 1:
    return TUNING_STEP_160M;
    break;
  case 3:
    return TUNING_STEP_80M;
    break;
  case 5:
    return TUNING_STEP_60M;
    break;
  case 7:
    return TUNING_STEP_40M;
    break;
  case 10:
    return TUNING_STEP_30M;
    break;
  case 14:
    return TUNING_STEP_20M;
    break;
  case 18:
    return TUNING_STEP_18M;
    break;
  case 21:
    return TUNING_STEP_15M;
    break;
  case 24:
    return TUNING_STEP_12M;
    break;
  case 30:
    return TUNING_STEP_10M;
    break;
  default:
    return 1;
    break;
  }
}

void reset_LC()
{
  c_side_ant = HIGH;
  c_side_trx = LOW;
  clearRegistersL();
  clearRegistersC();
  writeRegistersL();
  writeRegistersC();
}

void loop()
{
  frequency_lock = false;
  uint16_t tuning_step = 1;

  float swr = getSWR();
  if(swr == 0.0) {
    freq_lock = SlidingWindow(10);
  }

  if(FreqCount.available() && (swr != 0.0)) {
    freq_lock.update((((float)FreqCount.read() * 4096.0 / 100000.0)));
    if (frequency > 1.0 && frequency < 30.0) {
      if (freq_lock.getMaxDeviation() < 0.2) {
        if (freq_lock.getMean() > 1.0 && freq_lock.getMean() < 30.0) {
          frequency_lock = true;
          frequency = freq_lock.getMean();
        }
      }
    }
  }

  if(frequency_lock) {
    for (size_t i = 0; i < freq_vector.max_size(); i++)
    {
      if(abs(freq_vector.at(i).frq - frequency) < 0.01) {
        if(freq_vector.at(i).cap < 0){
          c_side_ant = HIGH;
          c_side_trx = LOW;
        } else {
          c_side_ant = LOW;
          c_side_trx = HIGH;
        }
        set_ind_value(freq_vector.at(i).ind);
        writeRegistersL();
        set_cap_value(abs(freq_vector.at(i).cap));
        writeRegistersC();
        tuned = true;
        tuned_frequency = frequency;
      }
    }
  }

  if(swr > MAX_SWR && (abs(frequency - tuned_frequency) > 0.07) && tuned && frequency_lock) {
    tuned_frequency = 999.9;
    tuned = false;
  }

  if (frequency_lock && !tuned)
  {
    tuning_step = set_tuning_step(frequency);
    grid_vector.clear();
    if (swr < MAX_TUNING_SWR)
    {
      reset_LC();
      if(coarse_ind(tuning_step) == 2) return;
      uint8_t coarse_cap_exit_code = coarse_cap(tuning_step);
      if(coarse_cap_exit_code == 2) return;
      if (coarse_cap_exit_code == 1)
      {
        c_side_ant = LOW;
        c_side_trx = HIGH;
        clearRegistersC();
        writeRegistersC();
        coarse_cap(tuning_step);
      }
      // Fine tuning
      for (int i = 3; i > 0; i--)
      {
        for (int y = 0; y < 3; y++)
        {
          uint8_t fine_tune_exit_code = fine_tuning(i); // one last pass to finetune
          if(fine_tune_exit_code == 2) return;
          if((fine_tune_exit_code == 1)) break;
          if (tuned) return;
        }
      }
      tuned = true;
      tuned_frequency = frequency;
      freq_vector.push_back({frequency, ind_value, (int16_t)(-1 + (2 * c_side_trx)) * (cap_value)});
      delay(2000);
    }
  }
}