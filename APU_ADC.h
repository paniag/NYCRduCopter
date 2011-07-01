#pragma once

/* 
 * Adapted from Arducopter project by Eric Paniagua. Original header comment below.
 *
 */

/*
	AP_ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
	Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Methods:
		Init() : Initialization of ADC. (interrupts etc)
		Ch(ch_num) : Return the ADC channel value

*/

class APU_ADC
{
public:
    APU_ADC() {}; // Constructor
    virtual void Init() {};
    virtual int Ch(unsigned char ch_num) = 0;
private:
};

#include "APU_ADC_Uno.h"
