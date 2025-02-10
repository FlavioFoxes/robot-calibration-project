#include "tricycle.h"

Tricycle::Tricycle(double timestamp, 
                   uint32_t tick_steer,
                   uint32_t tick_traction,
                   Vector3d pose):
    _timestamp(timestamp),
    _tick_steer(tick_steer),
    _tick_traction(tick_traction),
    _pose(pose){}


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

Vector3d Tricycle::get_pose()
{
    return _pose;
}

double Tricycle::get_steering_angle()
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
double Tricycle::encoder_to_angle(uint32_t tick)
{
    int64_t t = (tick > _max_steer / 2) ? (int64_t) tick - _max_steer : (int64_t) tick;
    return 2 * M_PI / _max_steer * t * _k_steer;
}

// protected
double Tricycle::encoder_to_meters(uint32_t tick, uint32_t next_tick)
{
    // Delta ticks between two consecutive timestamps
    int64_t delta_ticks = ((int64_t) next_tick - (int64_t) tick); 
    // Managing overflow
    if(delta_ticks < -100000){
        delta_ticks += UINT32_MAX;
    }
    return  delta_ticks * _k_traction / _max_traction;
}

// Step function
void Tricycle::step(uint32_t next_tick_traction, uint32_t actual_tick_steer)
{
    // Impose actual steering tick
    _tick_steer = actual_tick_steer;
    // Steering angle
    _steering_angle = encoder_to_angle(_tick_steer) + _steer_offset;
    // Total distance
    double s = encoder_to_meters(_tick_traction, next_tick_traction);

    double dtheta = s * sin(_steering_angle) / _baseline;
    double dx = s * cos(_steering_angle) * cos(dtheta);
    double dy = s * cos(_steering_angle) * sin(dtheta);

    // Next pose of KC is the product of the actual pose (w.r.t. world frame) and the movement
    Vector3d d_pose = Vector3d(dx, dy, dtheta);
    Vector3d next_pose = utils::t2v( utils::v2t(_pose)*utils::v2t(d_pose) );
    
    // Actual pose is the next one
    _pose = next_pose;

    // Sensor pose (w.r.t. World frame) is the product of the pose of the KC and the relative pose of sensor
    Vector3d sensor_pose = utils::t2v( utils::v2t(_pose)*utils::v2t(_sensor_pose_rel) );
    _sensor_pose = sensor_pose;

    // Actual traction tick is the next one
    _tick_traction = next_tick_traction;

    // Write in the file actual pose
    utils::write_pose(std::string("trajectories/model_pose_uncalibrated.txt"), _pose);
    utils::write_pose(std::string("trajectories/tracker_pose_uncalibrated.txt"), _sensor_pose);

}