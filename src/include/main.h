#include <FreqCount.h>
#include <Arduino.h>
#include <math.h>

void setup();               
void clearRegistersL(); 
void clearRegistersC();
void writeRegistersC();
void writeRegistersL();
void set_cap_value(int);
void set_ind_value(int);
void setRegisterPinL(int, int);
void setRegisterPinC(int, int);
float getSWR();
void coarse_ind(uint16_t);
void coarse_cap(uint16_t);
void loop();