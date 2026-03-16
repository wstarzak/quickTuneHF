#include "main.h"
#include "pinout.h"

#define MAX_SWR 1.3
#define MAX_TUNING_SWR 15.0

// ─── Diode nonlinearity correction ────────────────────────────────────────────
// Directional-coupler diodes operate in the square-law region at low RF levels
// (V_dc ∝ V_rf²) and transition toward linear detection at higher levels.
// Both detectors must be corrected identically so the ref/fwd ratio reflects
// true voltage ratios before the SWR formula is applied.
//
// Available methods — set SWR_CORRECTION_TYPE to one of:
//   SWR_CORRECTION_NONE  – raw ADC readings, no correction
//   SWR_CORRECTION_LUT   – lookup table with linear interpolation (recommended)
//   SWR_CORRECTION_SQRT  – square-root correction (pure square-law assumption)
//
// LUT notes:
//   • 33 breakpoints spaced SWR_LUT_STEP (32) ADC counts apart cover 0–1023.
//   • Default values were computed as corrected = sqrt(raw × 1023), which is
//     the ideal square-law correction normalised to full scale.
//   • To calibrate, apply a known 50-Ω source at several power levels, measure
//     the raw ADC values, and replace the corresponding table entries with the
//     ADC-equivalent values that yield the correct 1:1 ratio at each level.
//   • The table is stored in program flash (PROGMEM) to preserve SRAM.
// ──────────────────────────────────────────────────────────────────────────────

#define SWR_CORRECTION_NONE 0
#define SWR_CORRECTION_LUT  1
#define SWR_CORRECTION_SQRT 2

#define SWR_CORRECTION_TYPE SWR_CORRECTION_LUT

#define SWR_LUT_STEP 32
#define SWR_LUT_SIZE 33   // breakpoints at 0, 32, 64, …, 1024

// Default: pure square-law correction — corrected = sqrt(raw * 1023).
// Replace with empirically measured values for your specific diodes.
static const uint16_t swr_lut[SWR_LUT_SIZE] PROGMEM = {
      0,  181,  256,  313,  362,  404,  443,  479,
    512,  543,  572,  599,  627,  652,  677,  701,
    724,  745,  767,  788,  808,  828,  848,  866,
    885,  903,  922,  939,  957,  974,  991, 1007,
   1023
};

// Apply the selected nonlinearity correction to a single raw ADC sample.
static float correctDiodeReading(float raw)
{
#if SWR_CORRECTION_TYPE == SWR_CORRECTION_NONE
  return raw;

#elif SWR_CORRECTION_TYPE == SWR_CORRECTION_SQRT
  // Square-root correction assumes both detectors are fully in the square-law
  // region.  Output is normalised so that 1023 in → 1023 out.
  if (raw <= 0.0f) return 0.0f;
  return sqrtf(raw * 1023.0f);

#elif SWR_CORRECTION_TYPE == SWR_CORRECTION_LUT
  if (raw <= 0.0f)    return 0.0f;
  if (raw >= 1023.0f) return 1023.0f;
  uint16_t idx  = (uint16_t)(raw / SWR_LUT_STEP);
  float    frac = (raw - idx * SWR_LUT_STEP) / (float)SWR_LUT_STEP;
  float    lo   = (float)pgm_read_word(&swr_lut[idx]);
  float    hi   = (float)pgm_read_word(&swr_lut[idx + 1]);
  return lo + frac * (hi - lo);

#else
  #error "Unknown SWR_CORRECTION_TYPE – choose NONE, LUT, or SQRT"
#endif
}

#define L_COUNT 7
#define C_COUNT 8

#define DEBUG true

#if DEBUG
  #define DBG_PRINTF(fmt, ...) Serial.printf("[DBG] " fmt "\n", ##__VA_ARGS__)
#else
  #define DBG_PRINTF(fmt, ...)
#endif

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
  DBG_PRINTF("Setup complete");
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

float getSWR(uint8_t average = 5)
{
  float ref = 0, fwd = 0;
  uint8_t valid = 0;
  uint8_t attempts = 0;

  while (valid < average && attempts < average + 20)
  {
    float ref_tmp = correctDiodeReading((float)analogRead(SWR_REF_PIN));
    float fwd_tmp = correctDiodeReading((float)analogRead(SWR_FWD_PIN));
    attempts++;
    if (ref_tmp > fwd_tmp) {
      continue;
    }
    ref += ref_tmp;
    fwd += fwd_tmp;
    valid++;
  }
  
  if (ref < 10.0 && fwd < 10.0)
  {
    DBG_PRINTF("getSWR: no TX (ref=%.1f fwd=%.1f)", ref, fwd);
    return 0.0;
  }

  float swr_result = (1.0 + (ref / fwd)) / (1.0 - (ref / fwd));
  DBG_PRINTF("getSWR: ref=%.1f fwd=%.1f -> SWR=%.2f", ref, fwd, swr_result);
  return swr_result;
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

  DBG_PRINTF("coarse_ind: step=%u max=%u", ind_step, max_ind_val);
  for (uint16_t i = min_swr_ind_val; i < max_ind_val; i += ind_step)
  {
    set_ind_value(i);
    writeRegistersL();

    if (grid_vector.max_size() > grid_vector.size())
    {
      grid_vector.push_back({ind_value, (int16_t)(-1 + (2 * c_side_trx)) * cap_value});
    }

    float current_swr = getSWR();
    DBG_PRINTF("coarse_ind: ind=%u SWR=%.2f (min=%.2f)", ind_value, current_swr, min_swr);
    if (current_swr == 0.0) // TX stopped while tunnning
    {
      DBG_PRINTF("coarse_ind: TX stopped");
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
      DBG_PRINTF("coarse_ind: SWR rising, stopping early at ind=%u", ind_value);
      break;
    }
  }

  // At the end of tuning routine set inductor value with corresponded with minimum SWR
  set_ind_value(min_swr_ind_val);
  writeRegistersL();
  DBG_PRINTF("coarse_ind: best ind=%u SWR=%.2f exit=%u", min_swr_ind_val, min_swr, exit_code);
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

  DBG_PRINTF("coarse_cap: step=%u max=%u c_side_trx=%d", cap_step, max_cap_val, c_side_trx);
  for (uint16_t i = min_swr_cap_val; i < max_cap_val; i += cap_step)
  {
    set_cap_value(i);
    writeRegistersC();
    if (grid_vector.max_size() > grid_vector.size())
    {
      grid_vector.push_back({ind_value, (int16_t)(-1 + (2 * c_side_trx)) * cap_value});
    }

    float current_swr = getSWR();
    DBG_PRINTF("coarse_cap: cap=%d SWR=%.2f (min=%.2f)", cap_value, current_swr, min_swr);
    if (current_swr == 0.0) // TX stopped while tunnning
    {
      DBG_PRINTF("coarse_cap: TX stopped");
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
      DBG_PRINTF("coarse_cap: SWR rising, stopping early at cap=%d", cap_value);
      break;
    }
  }

  // At the end of tuning routine set capacitor value with corresponded with minimum SWR
  set_cap_value(min_swr_cap_val);
  writeRegistersC();
  DBG_PRINTF("coarse_cap: best cap=%d SWR=%.2f exit=%u", min_swr_cap_val, min_swr, exit_code);
  return exit_code;
}

// EXIT CODES
// 0 - succesfully decreased SWR
// 1 - failed to decrease
// 2 - TX stopped
uint8_t fine_tuning(int16_t fine_step = 1)
{
  uint16_t init_ind_val = ind_value;
  int16_t init_cap_val = cap_value;

  uint8_t exit_code = 1;

  float min_swr = 99.9;
  int16_t min_swr_cap_val = cap_value;
  uint16_t min_swr_ind_val = ind_value;
  
  // This checks all combinations around minimum value so we get best match
  for (int16_t ind = -fine_step; ind <= fine_step; ind += fine_step)
  {
    for (int16_t cap = -fine_step; cap <= fine_step; cap += fine_step)
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

      for (size_t i = 0; i < grid_vector.size(); i++)
      {
        if ((grid_vector.at(i).ind == (init_ind_val + ind)) && (grid_vector.at(i).cap == ((-1 + (2 * c_side_trx)) * (init_cap_val + cap))))
        {
          in_grid = true;
          break;
        }
      }

      if (!in_grid && grid_vector.max_size() > grid_vector.size())
      {
        grid_vector.push_back({(uint16_t)(init_ind_val + ind), (int16_t)(-1 + (2 * c_side_trx)) * (init_cap_val + cap)});
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
      DBG_PRINTF("fine_tuning: ind=%u cap=%d SWR=%.2f", ind_value, cap_value, current_swr);
      if (current_swr == 0.0)
      {
        DBG_PRINTF("fine_tuning: TX stopped");
        tuned = false;
        return 2;
      }

      if (current_swr < MAX_SWR)
      {
        DBG_PRINTF("fine_tuning: TUNED ind=%u cap=%d SWR=%.2f", ind_value, cap_value, current_swr);
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
  DBG_PRINTF("fine_tuning: best ind=%u cap=%d SWR=%.2f exit=%u", min_swr_ind_val, min_swr_cap_val, min_swr, exit_code);
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
  case 28:
  case 29:
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
      if (freq_lock.getMaxDeviation() < 0.2) {
        if (freq_lock.getMean() > 1.0 && freq_lock.getMean() < 30.0) {
          frequency_lock = true;
          frequency = freq_lock.getMean();
        }
    }
  }
  DBG_PRINTF("loop: frq=%.3f MHz lock=%d SWR=%.2f tuned=%d tuned_frq=%.3f", frequency, frequency_lock, swr, tuned, tuned_frequency);

  if(frequency_lock) {
    for (size_t i = 0; i < freq_vector.size(); i++)
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
    DBG_PRINTF("loop: frequency shifted (%.3f -> %.3f), clearing tuned state", tuned_frequency, frequency);
    tuned_frequency = 999.9;
    tuned = false;
  }

  if (frequency_lock && !tuned)
  {
    tuning_step = set_tuning_step(frequency);
    grid_vector.clear();
    DBG_PRINTF("loop: starting tuning frq=%.3f SWR=%.2f step=%u", frequency, swr, tuning_step);
    if (swr < MAX_TUNING_SWR)
    {
      reset_LC();
      if(coarse_ind(tuning_step) == 2) return;
      uint8_t coarse_cap_exit_code = coarse_cap(tuning_step);
      if(coarse_cap_exit_code == 2) return;
      if (coarse_cap_exit_code == 1)
      {
        DBG_PRINTF("loop: coarse_cap failed, retrying with c_side_trx=HIGH");
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
          DBG_PRINTF("loop: fine_tuning pass i=%d y=%d", i, y);
          uint8_t fine_tune_exit_code = fine_tuning(i); // one last pass to finetune
          if(fine_tune_exit_code == 2) return;
          if((fine_tune_exit_code == 1)) break;
          if (tuned) return;
        }
      }
      DBG_PRINTF("loop: tuning complete ind=%u cap=%d frq=%.3f", ind_value, cap_value, frequency);
      tuned = true;
      tuned_frequency = frequency;
      freq_vector.push_back({frequency, ind_value, (int16_t)(-1 + (2 * c_side_trx)) * (cap_value)});
      delay(2000);
    }
    else
    {
      DBG_PRINTF("loop: SWR=%.2f too high (max=%.1f), skipping tuning", swr, MAX_TUNING_SWR);
    }
  }
}