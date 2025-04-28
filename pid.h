/* This is ported from a PID controller created by https://gist.github.com/mattogodoy
original PID controller can be found here https://gist.github.com/mattogodoy/910ef7612950161f4a9871c09b62fec7
It was oringally writen in GDScript. */


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
