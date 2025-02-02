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
            dataset.time.push_back(stod(temp)); // ERROR
            
            s >> temp >> temp;
            dataset.ticks_steer.push_back(static_cast<uint32_t>(stoul(temp)));                
            s >> temp;
            dataset.ticks_traction.push_back(static_cast<uint32_t>(stoul(temp)));

            s >> temp >> temp;
            dataset.model_pose_x.push_back(stof(temp));
            
            s >> temp;
            dataset.model_pose_y.push_back(stof(temp));
            s >> temp;
            dataset.model_pose_theta.push_back(stof(temp));
            s >> temp >> temp;
            dataset.tracker_pose_x.push_back(stof(temp));
            s >> temp;
            dataset.tracker_pose_y.push_back(stof(temp));
            s >> temp;
            dataset.tracker_pose_theta.push_back(stof(temp));
            
        }
        // Close the file stream once all lines have been read
        file.close();
        return dataset;
    }
}
