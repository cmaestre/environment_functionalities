#ifndef __LIB_ENVIRONMENT_H__
#define __LIB_ENVIRONMENT_H__

#include <iostream>
#include <random>
#include <math.h>
#include <time.h>
#include <Eigen/Core>
#include <boost/timer.hpp>
#include <fstream>
#include <sys/param.h>
#include <cmath>
#include <string>
#include <tuple>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetLinkState.h>

#include "../../src/lib/env_values.hpp"


/**
 * @brief the function to spawn the environment
 * @param a
 * @param a
 * @return bool that
 */
bool spawn_environment(ros::ServiceClient& spawner,
                       ros::NodeHandle nh,
                       std::string project_name);

/**
 * @brief the function to spawn objects
 * @param name of the object to spawn, and its pose and the service client
 * @param argv
 * @return bool that sees if it succeed in spawning the model in gazebo or not
 */
bool spawn_model(std::string& model_to_spawn,
                 ros::ServiceClient& my_spawner,
                 geometry_msgs::Pose& model_pose,
                 std::string project_name);

/**
 * a
 */
geometry_msgs::Pose get_model_pose(ros::NodeHandle nh,
                                    std::string model_name);

/**
 * @brief remove the spawned models
 * @param a
 * @param a
 * @return bool that
 */
bool remove_environment(ros::ServiceClient& gazebo_model_delete,
                        ros::NodeHandle nh);

/**
 * @brief remove one model
 * @param a
 * @param a
 * @return bool that
 */
bool remove_model(std::string model_name,
                   ros::ServiceClient& gazebo_model_delete);

/**
 * @brief get spawned model values
 * @param a
 * @param a
 * @return bool that
 */
std::tuple<gazebo_msgs::GetModelState, double, double, double> get_object_values(
        std::string model_name,
        ros::ServiceClient& gazebo_model_state,
        bool robot_frame = false);

/**
 * @brief get link within spawned model values
 * @param a
 * @param a
 * @return bool that
 */
std::tuple<gazebo_msgs::GetLinkState, double, double, double> get_link_values(
        std::string link_name,
        ros::ServiceClient& gazebo_link_state);

/**
 * @brief define coordinates of all points (object sides, and intermediate points)
 * @param a
 * @param a
 * @return bool that
 */
void compute_predefined_scenario_wps(std::vector<Eigen::Vector3d> &side_position,
                                     std::vector<Eigen::Vector3d> &mid_point_position_vector,
                                     geometry_msgs::Pose& object_pose,
                                     Env_values env_values,
                                     bool random_values);

#endif /* __LIB_ENVIRONMENT_H__ */
