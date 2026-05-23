#include "mymath.h"
#include "Motors.h"
#include "vmc.h"



float Lowpass_Filter(float *last_output, float input, float alpha)
{
    *last_output = alpha * input + (1.0f - alpha) * (*last_output);
    return *last_output;
}
float out = 0;
    static int init = 0;
float target_ramp(float now, float target)
{
    
    float step = 0.35f;

    if (!init)
    {
        out = now;
        init = 1;
    }

    if (out < target)
    {
        out += step;
        if (out > target) out = target;
    }
    else if (out > target)
    {
        out -= step;
        if (out < target) out = target;
    }

    return out;
}

