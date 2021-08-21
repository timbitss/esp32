/**
 * @file RCFilter.h
 * @author Timothy Nguyen
 * @brief Simple IIR LPF using backward Euler transformation.
 * @version 0.1
 * @date 2021-08-19
 * 
 * Based off https://www.dsp-weimich.com/digital-signal-processing/iir-first-order-digital-filter/.
 *
 * To prevent aliasing, the cutoff frequency of the filter must be below the Nyquist frequency (1/2 the sampling frequency).
 */

#pragma once 

#include <cstdint>

class RCFilter
{
public:
    RCFilter(uint32_t sampling_freq_hz, float cutoff_freq_hz);
    
    float filter(float input_sig);

private:
    float A;
    float output; 
};