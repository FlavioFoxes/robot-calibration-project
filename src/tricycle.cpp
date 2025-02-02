#include "tricycle.h"

Tricycle::Tricycle(double timestamp, 
                   uint32_t tick_steer,
                   uint32_t tick_traction,
                   Vector3f position,
                   float orientation):
    _timestamp(timestamp),
    _tick_steer(tick_steer),
    _tick_traction(tick_traction),
    _position(position),
    _orientation(orientation){}





double Tricycle::get_timestamp()
{
    return _timestamp;
}

uint32_t Tricycle::get_tick_steer()
{
    return _tick_steer;
}

uint32_t Tricycle::get_tick_traction()
{
    return _tick_traction;
}

Vector3f Tricycle::get_position()
{
    return _position;
}

float Tricycle::get_orientation()
{
    return _orientation;
}

float Tricycle::get_steering_angle()
{
    return _steering_angle;
}

float Tricycle::get_k_steer()
{
    return _k_steer;
}

float Tricycle::get_k_traction()
{
    return _k_traction;
}

float Tricycle::get_baseline()
{
    return _baseline;
}

uint16_t Tricycle::get_max_steer()
{
    return _max_steer;
}

uint16_t Tricycle::get_max_traction()
{
    return _max_traction;
}

// protected
float Tricycle::encoder_to_angle(uint32_t tick)
{
    return 2 * M_PI / _max_steer * tick * _k_steer;
}

// protected
float Tricycle::encoder_to_meters(uint32_t tick, uint32_t next_tick)
{
    return 2*M_PI / _max_traction * (next_tick - tick) * _k_traction;
}

// Step function
void Tricycle::step(double next_timestamp, uint32_t next_tick_traction)
{
    float steering_angle = encoder_to_angle(_tick_steer);
    std::cout << steering_angle << std::endl;
}