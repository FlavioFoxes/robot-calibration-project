#include "tricycle.h"

Tricycle::Tricycle(uint32_t tick_steer,
                   uint32_t tick_traction,
                   Vector3d initial_model_pose,
                   Vector3d initial_sensor_pose) : 
    _tick_steer(tick_steer),
    _tick_traction(tick_traction),
    _pose(initial_model_pose),
    _sensor_pose(initial_sensor_pose) {}


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

Vector3d Tricycle::get_sensor_pose_rel()
{
    return _sensor_pose_rel;
}

void Tricycle::set_model_pose(Vector3d model_pose)
{
    _pose = model_pose;
}

void Tricycle::set_sensor_pose(Vector3d sensor_pose)
{
    _sensor_pose = sensor_pose;
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
    // If tick is greater than half a turn, consider it as negative tick
    int64_t t = (tick > _max_steer / 2) ? (int64_t)tick - _max_steer : (int64_t)tick;
    return 2 * M_PI / _max_steer * t * k_steer;
}

// protected
double Tricycle::encoder_to_meters(double k_traction, uint32_t tick, uint32_t next_tick)
{
    // Delta ticks between two consecutive timestamps
    int64_t delta_ticks = ((int64_t)next_tick - (int64_t)tick);
    // Managing overflow
    if (delta_ticks < -100000)
    {
        delta_ticks += UINT32_MAX;
    }
    return delta_ticks * k_traction / _max_traction;
}

void Tricycle::step(Vector3d d_pose, bool isCalibrated)
{
    // Displacement of the sensor (using static matrix product) as affine transformation
    Affine2d d_pose_sensor = utils::v2t(_sensor_pose_rel).inverse() * utils::v2t(d_pose) * utils::v2t(_sensor_pose_rel);
    // Next pose of the robot (applying robot displacement)
    Vector3d next_pose = utils::t2v(utils::v2t(_pose) * utils::v2t(d_pose));
    // Next pose of the sensor (applying sensor displacement)
    Vector3d sensor_pose = utils::t2v(utils::v2t(_sensor_pose) * d_pose_sensor);

    // Assign new model pose and new sensor pose
    _pose = next_pose;
    _sensor_pose = sensor_pose;

    
    // if (!isCalibrated)  // If parameters are uncalibrated
    // {
    //     // Save in file actual model pose and actual sensor pose
    //     utils::write_pose(std::string("trajectories/model_pose_uncalibrated.txt"), _pose);
    //     utils::write_pose(std::string("trajectories/tracker_pose_uncalibrated.txt"), _sensor_pose);
    // }
    // else 
    // {
    //     // Save in file actual calibrated model pose and actual calibrated sensor pose
    //     utils::write_pose(std::string("trajectories/model_pose_calibrated.txt"), _pose);
    //     utils::write_pose(std::string("trajectories/tracker_pose_calibrated.txt"), _sensor_pose);
    // }
}

Vector3d Tricycle::predict(std::vector<double> parameters,
                           Vector3d pose,
                           uint32_t actual_tick_traction,
                           uint32_t next_tick_traction,
                           uint32_t actual_tick_steer)
{
    // Unpack parameters
    double k_steer = parameters[0];
    double k_traction = parameters[1];
    double steer_offset = parameters[2];
    double baseline = parameters[3];
    Vector3d sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);

    // Convert steering encoder measurements in steering angle
    double steering_angle = encoder_to_angle(k_steer, actual_tick_steer) + steer_offset;

    // Convert traction encoder measurements in traveled distance
    double s = encoder_to_meters(k_traction, actual_tick_traction, next_tick_traction);

    // Save actual informations
    _tick_steer = actual_tick_steer;
    _tick_traction = actual_tick_traction;
    _steering_angle = steering_angle;

    // Compute displacements (ODE)
    double dtheta = s * sin(steering_angle) / baseline;
    double dx = s * cos(steering_angle) * cos(dtheta);
    double dy = s * cos(steering_angle) * sin(dtheta);
    Vector3d d_pose = Vector3d(dx, dy, dtheta);

    // DEBUG
    // std::cout << "-----DISPLACEMENT-----\n" << d_pose << std::endl;

    return d_pose;
}