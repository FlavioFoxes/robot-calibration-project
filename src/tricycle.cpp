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

double Tricycle::get_steer_offset()
{
    return _steer_offset;
}

double Tricycle::get_k_steer()
{
    return _k_steer;
}

double Tricycle::get_k_traction()
{
    return _k_traction;
}

double Tricycle::get_baseline()
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

Vector3d Tricycle::get_sensor_pose()
{
    return _sensor_pose;
}

void Tricycle::set_model_pose(Vector3d model_pose)
{
    _pose = model_pose;
}

void Tricycle::set_calibrated_parameters(std::vector<double> parameters)
{
    _k_steer = parameters[0];
    _k_traction = parameters[1];
    _steer_offset = parameters[2];
    _baseline = parameters[3];
    _sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);
}

std::vector<double> Tricycle::get_parameters_to_calibrate()
{
    double sensor_x = _sensor_pose_rel[0];
    double sensor_y = _sensor_pose_rel[1];
    double sensor_theta = _sensor_pose_rel[2];
    
    return std::vector<double>{_k_steer, _k_traction, _steer_offset, _baseline, sensor_x, sensor_y, sensor_theta};
}

// protected
double Tricycle::encoder_to_angle(double k_steer, uint32_t tick)
{
    int64_t t = (tick > _max_steer / 2) ? (int64_t) tick - _max_steer : (int64_t) tick;
    return 2 * M_PI / _max_steer * t * k_steer;
}

// protected
double Tricycle::encoder_to_meters(double k_traction, uint32_t tick, uint32_t next_tick)
{
    // Delta ticks between two consecutive timestamps
    int64_t delta_ticks = ((int64_t) next_tick - (int64_t) tick); 
    // Managing overflow
    if(delta_ticks < -100000){
        delta_ticks += UINT32_MAX;
    }
    return  delta_ticks * k_traction / _max_traction;
}

// Step function
// Make the tricycle forward of one step
void Tricycle::step(uint32_t actual_tick_traction, uint32_t next_tick_traction, uint32_t actual_tick_steer, bool isCalibrated)
{
    std::tuple<double, Vector3d, Vector3d> tuple = predict(get_parameters_to_calibrate(), actual_tick_traction, next_tick_traction, actual_tick_steer);
    double steering_angle = std::get<0>(tuple);
    Vector3d next_pose = std::get<1>(tuple);
    Vector3d sensor_pose = std::get<2>(tuple);

    // Impose actual steering tick
    _tick_steer = actual_tick_steer;
    _tick_traction = actual_tick_traction;

    // Assign steering angle
    _steering_angle = steering_angle;
    // Assign model pose and sensor pose
    _pose = next_pose;
    _sensor_pose = sensor_pose;

    // Actual traction tick is the next one

    if(!isCalibrated){
        // // Save in file actual model pose and actual sensor pose
        utils::write_pose(std::string("trajectories/model_pose_uncalibrated.txt"), _pose);
        utils::write_pose(std::string("trajectories/tracker_pose_uncalibrated.txt"), _sensor_pose);
    }
    else{
        utils::write_pose(std::string("trajectories/model_pose_calibrated.txt"), _pose);
        utils::write_pose(std::string("trajectories/tracker_pose_calibrated.txt"), _sensor_pose);
    }
  
}

std::tuple<double, Vector3d, Vector3d> Tricycle::predict(std::vector<double> parameters,
                                                         uint32_t tick_traction,
                                                         uint32_t next_tick_traction, 
                                                         uint32_t tick_steer)
{
    double k_steer = parameters[0];
    double k_traction = parameters[1];
    double steer_offset = parameters[2];
    double baseline = parameters[3];
    Vector3d sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);

    double steering_angle = encoder_to_angle(k_steer, tick_steer) + steer_offset;
    // Total distance
    double s = encoder_to_meters(k_traction, tick_traction, next_tick_traction);

    double dtheta = s * sin(steering_angle) / baseline;
    double dx = s * cos(steering_angle) * cos(dtheta);
    double dy = s * cos(steering_angle) * sin(dtheta);

    // Next pose of KC is the product of the actual pose (w.r.t. world frame) and the movement
    Vector3d d_pose = Vector3d(dx, dy, dtheta);
    Vector3d next_pose = utils::t2v( utils::v2t(_pose)*utils::v2t(d_pose) );
    // Sensor pose (w.r.t. World frame) is the product of the pose of the KC and the relative pose of sensor
    Vector3d sensor_pose = utils::t2v( utils::v2t(next_pose)*utils::v2t(sensor_pose_rel) );

    return std::tuple<double, Vector3d, Vector3d>(steering_angle, next_pose, sensor_pose);
}
