#include "../../include/environment_functionalities/lib_environment.hpp"
#include "environment_functionalities/RestartWorld.h"
#include <iostream>

Env_values env_values;

int restart_environment(environment_functionalities::RestartWorld::Request &req,
                  environment_functionalities::RestartWorld::Response &res,
                  ros::NodeHandle& nh){

    bool real_robot;
    nh.getParam("real_robot", real_robot);
    if (real_robot){
        char line[25];
        std::cout << " Restart manually the environment and press ENTER \n>";
        std::cin.get( line, 25 );
    } 

    else {
        ROS_INFO("Establish communication tools");
        ros::ServiceClient gazebo_spawn_clt = nh.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
        ros::ServiceClient gazebo_model_delete = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        // Required for communication with moveit components
        ros::AsyncSpinner spinner (1);
        spinner.start();

        std::string option = req.option;
        if (strcmp(option.c_str(), "setup") == 0){

            // Remove setup objects (if available)
            ROS_INFO("REMOVE SETUP");
            bool remove_spawn_res = remove_environment(gazebo_model_delete, nh);
            if (!remove_spawn_res){
                ROS_ERROR_STREAM("restart_environment : Problem removing setup");
                res.success = false;
                return 0;
            }

            //spawn the table then the cube
            ROS_INFO("SPAWN");
            std::string project_name = env_values.get_project_name();
            bool spawn_res = spawn_environment(gazebo_spawn_clt, nh, project_name);
            if (!spawn_res){
                ROS_ERROR_STREAM("restart_environment : Problem spawning models");
                res.success = false;
                return 0;
            }
        } else if (strcmp(option.c_str(), "object") == 0){
            // Remove setup objects (if available)
            ROS_INFO("REMOVE OBJECT");
            std::string model_name = req.model_name;
            bool remove_spawn_res = remove_model(model_name,
                                                 gazebo_model_delete);
            if (!remove_spawn_res){
                ROS_ERROR_STREAM("restart_environment : Problem removing object");
                res.success = false;
                return 0;
            }

            // Update object values
            geometry_msgs::Pose current_model_pose = get_model_pose(nh, model_name);
            current_model_pose.position.x = req.model_pos_x;
            current_model_pose.position.y = req.model_pos_y;
            current_model_pose.position.z = req.model_pos_z;

            // Spawn the object
            ROS_INFO("SPAWN");
            bool spawn_res = spawn_model(model_name,
                                         gazebo_spawn_clt,
                                         current_model_pose,
                                         env_values.get_project_name());
            if (!spawn_res){
                ROS_ERROR_STREAM("restart_environment : Problem spawning a model");
                res.success = false;
                return 0;
            }

        } else {
            ROS_ERROR_STREAM("restart_environment : Wrong option selected");
            res.success = false;
            return 0;
        }    

        res.success = true;
        spinner.stop();
    }

    ROS_INFO("Done!");
    return 1;
}

int main(int argc, char** argv)
{

    // Initialize ROS
    ros::init(argc, argv, "restart_environment_node");
    ros::NodeHandle nh;   

    ros::ServiceServer service = nh.advertiseService<environment_functionalities::RestartWorld::Request,
            environment_functionalities::RestartWorld::Response>("env/restart_environment", 
                boost::bind(restart_environment, _1, _2, nh));
    ROS_INFO("Ready to restart the environment.");
    ros::spin();

    return 0;
}
