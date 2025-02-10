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
    std::cout << "tick:     " << tick << std::endl;
    // Delta ticks between two consecutive timestamps
    int64_t delta_ticks = ((int64_t) next_tick - (int64_t) tick); 
    // std::cout << "delta ticks:      " << delta_ticks << std::endl;   // SEEMS TO WORK
    // Managing overflow
    if(delta_ticks < -100000){
        delta_ticks += UINT32_MAX;
    }
    return  delta_ticks * _k_traction / _max_traction;
}



void Tricycle::write_tricycle_pose(std::string name_file)
{
    std::ofstream pose_file(name_file, std::ofstream::out | std::ofstream::app);
    ASSERT(pose_file.is_open(), "Unable to open writing file!");
    Vector3d pose = get_pose();
    pose_file << pose[0] << ' ' << pose[1] << ' ' << pose[2] << std::endl;
    pose_file.close();
}

// Step function
void Tricycle::step(uint32_t next_tick_traction, uint32_t actual_tick_steer)
{
    // Impose actual steering tick
    _tick_steer = actual_tick_steer;
    // Steering angle
    _steering_angle = encoder_to_angle(_tick_steer);
    // Total distance
    double s = encoder_to_meters(_tick_traction, next_tick_traction);

    double dtheta = s * sin(_steering_angle) / _baseline;
    double dx = s * cos(_steering_angle) * cos(dtheta);
    double dy = s * cos(_steering_angle) * sin(dtheta);

    Vector3d d_pose = Vector3d(dx, dy, dtheta);

    Vector3d next_pose = utils::t2v( utils::v2t(_pose)*utils::v2t(d_pose) );
    _pose = next_pose;

    // _pose += Vector3d(dx, dy, dtheta);

    _tick_traction = next_tick_traction;
    write_tricycle_pose(std::string("example.txt"));

}