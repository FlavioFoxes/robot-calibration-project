#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"

using namespace Eigen;

class Tricycle{
    protected:
        double   _timestamp;
        uint32_t _tick_steer;
        uint32_t _tick_traction;
        // Kinematic center
        Vector3f _position;         // kc_x, kc_y, 0
        float    _orientation;      // theta_z
        float    _steering_angle;      // delta

        // Sensor
        Vector3f _sensor_position;
        float _sensor_orientation;

        float _k_steer = 0.1;
        float _k_traction = 0.0106141;
        float _baseline = 1.4;
        float _steer_offset = 0;

        uint16_t _max_steer = 8192;
        uint16_t _max_traction = 5000;

        // TODO: aggiungi posizioni/rotazioni tra sensore e kc    

        // Pass from encoder readers to measures
        float encoder_to_angle(uint32_t tick);
        float encoder_to_meters(uint32_t tick, uint32_t next_tick);

    public:
    
        // Constructor
        Tricycle(double timestamp, 
                 uint32_t tick_steer, 
                 uint32_t tick_traction, 
                 Vector3f position,
                 float orientation);

        // Getter information functions
        double   get_timestamp();
        uint32_t get_tick_steer();
        uint32_t get_tick_traction();
        Vector3f get_position();  // kc_x, kc_y, 0
        float    get_orientation();  // theta_z
        Vector3f get_sensor_position();  // kc_x, kc_y, 0
        float    get_sensor_orientation();  // theta_z
        float    get_steering_angle();  // delta
        float    get_k_steer();
        float    get_k_traction();
        float    get_baseline();
        uint16_t get_max_steer();
        uint16_t get_max_traction();
        
        // Step function (ODE)
        void step(uint32_t next_tick_traction);
        void write_tricycle_pose(std::string name_file);
};