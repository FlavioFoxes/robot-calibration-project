#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"

using namespace Eigen;

class Tricycle{
    protected:
        /*
        Actual information about the tricycle.
        In details:
            - _tick_steer:      reading of steering (absolute) encoder
            - _tick_traction:   reading of traction (incremental) encoder
            - _pose:            pose (x, y, theta) of the robot
            - _sensor_pose:     pose (x, y, theta) of the sensor
            - _steering_angle:  steering angle of the front wheel 
        */
        uint32_t _tick_steer;
        uint32_t _tick_traction;
        Vector3d _pose;                 // kc_x, kc_y, theta
        Vector3d _sensor_pose;
        double   _steering_angle;       // delta

        /*
        Kinematic parameters of the robot (to calibrate).
        Initialized with nominal values.
        In detail:
            - _k_steer:         how many radians correspond to one tick
            - _k_traction:      how many meter correspond to one tick
            - _steer_offset:    at which angle correspond the zero of the wheel
            - _baseline:        lenght of the base_line
        */
        double _k_steer = 0.1;
        double _k_traction = 0.0106141;
        double _steer_offset = 0;
        double _baseline = 1.4;

        /*
        Max ranges of encoders
        */
        const uint16_t _max_steer = 8192;
        const uint16_t _max_traction = 5000;

        /*
        Relative pose (x, y, theta) of the sensor w.r.t. robot kinematic center (to calibrate)
        */
        Vector3d _sensor_pose_rel = Vector3d(1.5, 0, 0);

        // Convert steering encoder measure to steering angle 
        double encoder_to_angle(double k_steer, uint32_t tick);
        
        /*
        Convert traction encoder measures to traveled distance
        NOTE: it takes in input actual traction tick and next traction tick,
              because it's an incremental encoder
        */ 
        double encoder_to_meters(double k_traction, uint32_t tick, uint32_t next_tick);


    public:
    
        // Constructor
        Tricycle(uint32_t tick_steer, 
                 uint32_t tick_traction, 
                 Vector3d initial_model_pose,
                 Vector3d initial_sensor_pose);

        // Getter information functions
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

        /*
        Returns the parameters must be calibrated in a std::vector.
        It is useful for numeric jacobian
        */
        std::vector<double> get_parameters_to_calibrate();
        
        // Setter information functions
        void set_model_pose(Vector3d model_pose);
        void set_sensor_pose(Vector3d sensor_pose);
        void set_calibrated_parameters(std::vector<double> parameters);
        
        /*
        Predict function (ODE).
        Predict the displacement of the robot, according to encoder measurements
        and parameters to calibrate.
        NOTE: parameters is the std::vector coming from get_parameters_to_calibrate().
              It is not possible to call it directly inside the function because of the
              usage in the error and jacobian computations.
        */
        Vector3d predict(std::vector<double> parameters,
                         Vector3d pose,
                         uint32_t actual_tick_traction,
                         uint32_t next_tick_traction, 
                         uint32_t actual_tick_steer);
        
        /*
        Step function. 
        Apply predicted displacement to robot pose and sensor pose.
        NOTE: isCalibrated param is used for trajectory generation 
        */
        void step(Vector3d d_pose, bool isCalibrated);
        
};