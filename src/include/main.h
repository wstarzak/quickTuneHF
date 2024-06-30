#include <FreqCount.h>
#include <Arduino.h>
#include <math.h>

void lock_frequency();
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
bool coarse_ind(int);
bool coarse_cap(int);
bool fine_cap();
bool fine_ind();
bool fine_tuning();
void set_cap_side();
void loop();