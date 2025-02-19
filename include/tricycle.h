#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"

using namespace Eigen;

class Tricycle{
    protected:
        double   _timestamp;
        uint32_t _tick_steer;
        uint32_t _tick_traction;
        Vector3d _pose;                 // kc_x, kc_y, theta
        Vector3d _sensor_pose;
        double   _steering_angle;       // delta

        double _k_steer = 0.1;
        double _k_traction = 0.0106141;
        double _baseline = 1.4;
        double _steer_offset = 0;

        const uint16_t _max_steer = 8192;
        const uint16_t _max_traction = 5000;

        Vector3d _sensor_pose_rel = Vector3d(1.5, 0, 0);    // x,y,theta w.r.t. baselink
                                                                // It can be converted using v2t

        // Pass from encoder readers to measures
        double encoder_to_angle(double k_steer, uint32_t tick);
        double encoder_to_meters(double k_traction, uint32_t tick, uint32_t next_tick);


    public:
    
        // Constructor
        Tricycle(double timestamp, 
                 uint32_t tick_steer, 
                 uint32_t tick_traction, 
                 Vector3d pose);

        // Getter information functions
        double    get_timestamp();
        uint32_t  get_tick_steer();
        uint32_t  get_tick_traction();
        Vector3d  get_pose();  // kc_x, kc_y, 0
        double    get_steering_angle();  // delta
        double    get_steer_offset();
        double    get_k_steer();
        double    get_k_traction();
        double    get_baseline();
        uint16_t  get_max_steer();
        uint16_t  get_max_traction();
        Vector3d  get_sensor_pose();
        Vector3d  get_sensor_pose_rel();

        void set_model_pose(Vector3d model_pose);
        void set_calibrated_parameters(std::vector<double> parameters);

        // Get parameters to calibrate in a std::vector.
        // It is useful for numeric jacobian
        std::vector<double> get_parameters_to_calibrate();
        
        // Step function (ODE). 
        // Recalls predict function and assign outputs to protected variables inside the class
        void step(uint32_t actual_tick_traction, uint32_t next_tick_traction, uint32_t tick_steer, bool isCalibrated);
        
        // Predict function (ODE)
        // Argument "parameters" are parameters to calibrate in vector format    
        std::tuple<double, Vector3d, Vector3d> predict(std::vector<double> parameters,
                                                       Vector3d pose,
                                                       uint32_t tick_traction,
                                                       uint32_t next_tick_traction, 
                                                       uint32_t tick_steer);
};