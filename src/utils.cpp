#include "utils.h"



namespace utils{

    // Read the file name_file
    Dataset create_dataset(string name_file){
        ifstream file(name_file);
        Dataset dataset;
        // String to store each line of the file.
        string line;

        ASSERT(file.is_open(), "Unable to open file");

        int i = 0;
        // Read each line from the file and store it in line
        while (getline(file, line)) {
            if(i<10){
                ++i;
                continue;
            }
            stringstream s(line);
            string temp;
            s >> temp >> temp;
            dataset.time.push_back(stod(temp)); 
            
            s >> temp >> temp;
            dataset.ticks_steer.push_back(static_cast<uint32_t>(stoul(temp)));                
            s >> temp;
            dataset.ticks_traction.push_back(static_cast<uint32_t>(stoul(temp)));

            s >> temp >> temp;
            dataset.model_pose_x.push_back(stod(temp));
            
            s >> temp;
            dataset.model_pose_y.push_back(stod(temp));
            s >> temp;
            dataset.model_pose_theta.push_back(stod(temp));
            s >> temp >> temp;
            dataset.tracker_pose_x.push_back(stod(temp));
            s >> temp;
            dataset.tracker_pose_y.push_back(stod(temp));
            s >> temp;
            dataset.tracker_pose_theta.push_back(stod(temp));
            
        }
        // Close the file stream once all lines have been read
        file.close();
        return dataset;
    }

    void write_pose(std::string name_file, Vector3d pose)
    {
        std::ofstream pose_file(name_file, std::ofstream::out | std::ofstream::app);
        ASSERT(pose_file.is_open(), "Unable to open writing file!");
        pose_file << pose[0] << ' ' << pose[1] << ' ' << pose[2] << std::endl;
        pose_file.close();
    }

    Affine2d v2t(Vector3d pose)
    {
        Affine2d t = Affine2d::Identity();
        t.translation() = Vector2d(pose[0], pose[1]);
        t.rotate(pose[2]);
        return t;
    }
    Vector3d t2v(Affine2d t)
    {
        Vector2d translation = t.translation();
        double angle = Rotation2Dd(t.rotation()).angle();
        return Vector3d(translation[0], translation[1], angle);
    }
}
