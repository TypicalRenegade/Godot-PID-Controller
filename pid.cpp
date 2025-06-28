/*
 *
 * pid.cpp
 *
 *
 * MIT License
 *
 * Copyright (c) 2025 TypicalRenegade
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
*/

#include "pid.h"

#include "core/math/math_funcs.h"

static float Clamp(float v, float min, float max) {
  return (v < min) ? min : (v > max) ? max : v;
}


float PID::update(float current_value, float target_value, float delta) {
    if (delta <= 0.0) {
        return 0.0;
    }

    float error = target_value - current_value;

    // calculate P term
    float P = proportional_gain * error;

    // calculate I term
    PID::integration_stored = Clamp(PID::integration_stored + (error * delta), -PID::integral_saturation, PID::integral_saturation);
    float I = integral_gain * PID::integration_stored;

    // calculate both D terms
    float error_rate_of_change = (error - PID::error_last) / delta;
    PID::error_last = error;

    float value_rate_of_change = (current_value - PID::value_last) / delta;
    PID::value_last = current_value;

    // choose D term to use
    float derive_measure = 0.0;

    if (derivative_initialized) {
        if (derive_measure == PID::DerivativeMeasurement::VELOCITY) {
            derive_measure = -value_rate_of_change;
        }

        else {
            derive_measure = error_rate_of_change;
        }
    }
    else {
        derivative_initialized = true;
    }

    float D = derivative_gain * derive_measure;

    float result = P + I + D;

    return Clamp(result, PID::output_min, PID::output_max);
}

float PID::update_angle(float current_angle, float target_angle, float delta) {
    if (delta <= 0) {
        return 0.0;
    }

    float error = _angle_difference(target_angle, current_angle);

    // calulate P term
    float P = PID::proportional_gain * error;

    // calculate I term
    PID::integration_stored = Clamp(PID::integration_stored + (error * delta), -PID::integral_saturation, PID::integral_saturation);
    float I = PID::integral_gain * PID::integration_stored;

    // calculate both D terms
    float error_rate_of_change = _angle_difference(error, PID::error_last) / delta;
    PID::error_last = error;

    float value_rate_of_change = _angle_difference(current_angle, PID::value_last) / delta;
    PID::value_last = current_angle;

    // choose D term to use
    float derive_measure = 0.0;

    if (derivative_initialized) {
        if (PID::derivative_measurement == PID::DerivativeMeasurement::VELOCITY) {
            derive_measure = -value_rate_of_change;
        }
        else {
            derive_measure = error_rate_of_change;
        }
    }
    else {
        PID::derivative_initialized = true;
    }

    float D = PID::derivative_gain * derive_measure;

    float result = P + I + D;

    return Clamp(result, PID::output_min, PID::output_max);
}


void PID::_reset() {
    derivative_initialized = false;
}
float PID::_angle_difference(float a, float b) {
    return Math::fmod((a - b + 540.0), 360.0) - 180.0; // calculate modular differance,
    // and remap to [-180, 180]
}


void PID::_bind_methods() {
    ClassDB::bind_method(D_METHOD("update", "current_value", "target_value", "delta"), &PID::update);
    ClassDB::bind_method(D_METHOD("update_angle", "current_angle", "target_angle", "delta"), &PID::update_angle);
    ClassDB::bind_method(D_METHOD("_reset"), &PID::_reset);
    ClassDB::bind_method(D_METHOD("_angle_difference", "a", "b"), &PID::_angle_difference);
    ClassDB::bind_method(D_METHOD("get_proportional_gain"), &PID::get_proportional_gain);
    ClassDB::bind_method(D_METHOD("get_integral_gain"), &PID::get_integral_gain);
    ClassDB::bind_method(D_METHOD("get_derivative_gain"), &PID::get_derivative_gain);
    ClassDB::bind_method(D_METHOD("get_output_min"), &PID::get_output_min);
    ClassDB::bind_method(D_METHOD("get_output_max"), &PID::get_output_max);
    ClassDB::bind_method(D_METHOD("get_integral_saturation"), &PID::get_integral_saturation);
    ClassDB::bind_method(D_METHOD("set_proportional_gain", "gain"), &PID::set_proportional_gain);
    ClassDB::bind_method(D_METHOD("set_integral_gain", "gain"), &PID::set_integral_gain);
    ClassDB::bind_method(D_METHOD("set_derivative_gain", "gain"), &PID::set_derivative_gain);
    ClassDB::bind_method(D_METHOD("set_output_min", "new_min"), &PID::set_output_min);
    ClassDB::bind_method(D_METHOD("set_output_max", "new_max"), &PID::set_output_max);
    ClassDB::bind_method(D_METHOD("set_integral_saturation", "saturation"), &PID::set_integral_saturation);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "proportional_gain", PROPERTY_HINT_RANGE, "0,1,0.1"), "set_proportional_gain", "get_proportional_gain");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "integral_gain", PROPERTY_HINT_RANGE, "0, 1, 0.1"), "set_integral_gain", "get_integral_gain");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "derivative_gain", PROPERTY_HINT_RANGE, "0, 1, 0.1"), "set_derivative_gain", "get_derivative_gain");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "output_min", PROPERTY_HINT_RANGE, "-2000, -1, 0.1"), "set_output_min", "get_output_min");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "output_max", PROPERTY_HINT_NONE, "1, 2000, 0.1"), "set_output_max", "get_output_max");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "integral_saturation", PROPERTY_HINT_NONE), "set_integral_saturation", "get_integral_saturation");

}

real_t PID::get_proportional_gain() const {
    return proportional_gain;
}

real_t PID::get_integral_gain() const {
    return integral_gain;
}

real_t PID::get_derivative_gain() const {
    return derivative_gain;
}

real_t PID::get_output_min() const {
    return output_min;
}

real_t PID::get_output_max() const {
    return output_max;
}

real_t PID::get_integral_saturation() const {
    return integral_saturation;
}

void PID::set_proportional_gain(real_t gain) {
    proportional_gain = gain;
}

void PID::set_integral_gain(real_t gain) {
    integral_gain = gain;
}

void PID::set_derivative_gain(real_t gain) {
    derivative_gain = gain;
}

void PID::set_output_min(real_t new_min) {
    output_min = new_min;
}

void PID::set_output_max(real_t new_max) {
    output_max = new_max;
}

void PID::set_integral_saturation(real_t saturation) {
    integral_saturation = saturation;
}
