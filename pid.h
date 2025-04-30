/* pid.h
 *
 * This is ported from a PID controller created by https://gist.github.com/mattogodoy original PID controller can be found *here https://gist.github.com/mattogodoy/910ef7612950161f4a9871c09b62fec7
 * It was oringally writen in GDScript.
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


#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED


#include "scene/main/node.h"

class PID : public Node {
    GDCLASS(PID, Node)

protected:
    // Functions & Variables?
    enum DerivativeMeasurement {
    VELOCITY,
    ERROR_RATE_OF_CHANGE
    };

public:
    float proportional_gain = 1.0;
    float integral_gain = 0.0;
    float derivative_gain = 0.5;

    float output_min = -1.0;
    float output_max = 1.0;
    float integral_saturation = 0.0;
    DerivativeMeasurement derivative_measurement = DerivativeMeasurement::VELOCITY;

protected:
    float value_last = 0.0;
    float error_last = 0.0;
    float integration_stored = 0.0;
    bool derivative_initialized = false;

public:
    float update(float current_value, float target_value, float delta);
    float update_angle(float current_angle, float target_angle, float delta);

    real_t get_proportional_gain() const;
    real_t get_integral_gain() const;
    real_t get_derivative_gain() const;
    real_t get_output_min() const;
    real_t get_output_max() const;
    real_t get_integral_saturation() const;
    void set_proportional_gain(real_t gain);
    void set_integral_gain(real_t gain);
    void set_derivative_gain(real_t gain);
    void set_output_min(real_t new_min);
    void set_output_max(real_t new_max);
    void set_integral_saturation(real_t saturation);

private:
    void _reset();
    float _angle_difference(float a, float b);

protected:
    static void _bind_methods();

};

#endif // PID_H_INCLUDED
