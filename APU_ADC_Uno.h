#pragma once

#include "APU_ADC.h"
#include "WProgram.h"

class APU_ADC_Uno : public APU_ADC
{
public:
    APU_ADC_Uno () {}
    virtual void init () {}
    virtual int Ch(unsigned char ch_num) { return analogRead(ch_num); }
private:
};
