#include <FreqCount.h>
#include <Arduino.h>
#include <math.h>
#include <Vector.h>
#include <SlidingWindow.h>

void setup();               
void clearRegistersL(); 
void clearRegistersC();
void writeRegistersC();
void writeRegistersL();
void set_cap_value(int);
void set_ind_value(int);
void setRegisterPinL(int, int);
void setRegisterPinC(int, int);
float getSWR(uint8_t);
uint8_t coarse_ind(uint16_t);
uint8_t coarse_cap(uint16_t);
void loop();