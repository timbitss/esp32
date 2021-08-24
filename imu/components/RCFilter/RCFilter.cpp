/**
 * @file RCFilter.cpp
 * @author Timothy Nguyen
 * @brief Simple IIR LPF using backward Euler transformation.
 * @version 0.1
 * @date 2021-08-19
 * 
 * Based off https://www.dsp-weimich.com/digital-signal-processing/iir-first-order-digital-filter/.
 */

#include "RCFilter.h"
#include "esp_log.h"

#define PI 3.14159265f

static const char* TAG = "RCFilter";

/**
 * @brief Construct a new RCFilter::RCFilter object.
 * 
 * @param sampling_freq Sampling frequency in Hz.
 * @param cutoff_freq   Cut off frequency in Hz.
 * @note To prevent aliasing, the cutoff frequency of the filter must be below the Nyquist frequency.
 * 
 * Tip: Smaller cutoff frequency = more filtering = more delay. 
 */
RCFilter::RCFilter(uint32_t sampling_freq_hz, float cutoff_freq_hz): A{1.0f}, output{0.0f}
{
    if(cutoff_freq_hz != 0.0f)
    {
        float RC = 1 / (2 * PI * cutoff_freq_hz);
        float Ts = 1.0f / sampling_freq_hz;
        A = Ts / (Ts + RC);
    }
    else
    {
        ESP_LOGI(TAG, "Cut-off frequency can not = 0 Hz.");
    }
    ESP_LOGI(TAG, "RC Filter constructed.");
}

/**
 * @brief Filter signal using IIR Filter.
 * 
 * @param input_sig Input signal.
 * @return float Filtered signal. 
 */
float RCFilter::filter(float input_sig)
{
    // Equivalent to Y[n] = (1-A) * Y[n-1] + A * X[n]
    output = output + A * (input_sig - output); 
    return output;
}