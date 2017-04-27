#ifndef __ENV_VALUES_HPP__
#define __ENV_VALUES_HPP__

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <Eigen/Core>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

struct Object_values {
    double cube_side = 0.1;
    double mid_point_dist = 0.15; // dist between mid points
};

class Env_values{

public:
    Object_values object_values;

    std::string project_name = "environment_functionalities";
    std::string get_project_name(){
        return project_name;
    }

    std::vector<std::string> cube_sides = {"right", "left", "up", "down", "no_side"}; //the enumeration is ordinated to link cube side with corresponding outside point
    std::vector<std::string> mid_points = {"right_mid", "left_mid", "front_mid", "back_mid"};

    std::string get_cube_side_value (int pos){
        return cube_sides[pos];
    }

    double get_cube_side(){
        return object_values.cube_side;
    }    

    std::string get_mid_points_value (int pos){
        return mid_points[pos];
    }

    double get_mid_point_dist(){
        return object_values.mid_point_dist;
    }    

};

#endif
