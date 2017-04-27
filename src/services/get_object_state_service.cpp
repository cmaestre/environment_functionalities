#include "../../include/environment_functionalities/lib_environment.hpp"
#include "environment_functionalities/GetObjectState.h"

bool get_state(environment_functionalities::GetObjectState::Request &req,
               environment_functionalities::GetObjectState::Response &res,
               ros::NodeHandle& nh){

    bool real_robot;
    nh.getParam("real_robot", real_robot);
    if (real_robot){
      res.object_state = {1000, 
                         1001, 
                         1002, 
                         1003, 
                         1004, 
                         1005};      
    } 
    else 
    {
      ros::ServiceClient gazebo_object_state = 
          nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

      // Required for communication with moveit components
      ros::AsyncSpinner spinner (1);
      spinner.start();

      auto cube_values = get_object_values(req.object_name,
                                           gazebo_object_state,
                                           true); //robot_frame
      gazebo_msgs::GetModelState the_state = std::get<0>(cube_values);
      double roll = std::get<1>(cube_values),
              pitch = std::get<2>(cube_values),
              yaw = std::get<3>(cube_values);
      res.object_state = {the_state.response.pose.position.x, 
                        the_state.response.pose.position.y, 
                        the_state.response.pose.position.z, 
                        roll, 
                        pitch, 
                        yaw};
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_object_state_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService<environment_functionalities::GetObjectState::Request,
          environment_functionalities::GetObjectState::Response>("env/get_object_state", boost::bind(get_state, _1, _2, n));
  ROS_INFO("Ready to get models states.");
  ros::spin();

  return 0;
}
