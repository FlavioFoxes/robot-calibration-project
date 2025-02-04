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

Vector3f Tricycle::get_sensor_position(){
    return _sensor_position;
};  

float Tricycle::get_sensor_orientation(){
    return _sensor_orientation;
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

void Tricycle::write_tricycle_pose(std::string name_file)
{
    std::ofstream pose_file(name_file, std::ofstream::out | std::ofstream::app);
    ASSERT(pose_file.is_open(), "Unable to open writing file!");
    Vector3f position = get_position();
    float orientation = get_orientation();
    pose_file << position.x() << ' ' << position.y() << ' ' << orientation << std::endl;
    pose_file.close();
}

// Step function
void Tricycle::step(uint32_t next_tick_traction)
{
    // Steering angle
    _steering_angle = encoder_to_angle(_tick_steer);
    // Total distance
    float s = encoder_to_meters(_tick_traction, next_tick_traction);
    // std::cout << s << std::endl;

    float dx = s * cos(_steering_angle) * cos(_orientation);
    float dy = s * cos(_steering_angle) * sin(_orientation);
    float dtheta = s * sin(_steering_angle) / _baseline;

    _position += Vector3f(dx, dy, 0.f);
    std::cout << "position:     " << _position << std::endl;
    _orientation += dtheta;
    _tick_traction = next_tick_traction;
    write_tricycle_pose(std::string("example.txt"));

}