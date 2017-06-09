#include "../../include/environment_functionalities/lib_environment.hpp"
#include <string>


/**
 * @brief spawn the environment objects
 * @param a
 * @param a
 * @return bool that
 */
bool spawn_environment(ros::ServiceClient& spawner,
                       ros::NodeHandle nh,
                       std::string project_name){
    std::vector<std::string> object_name_vector;
    nh.getParam("obj_name_vector", object_name_vector);
    int nb_models = object_name_vector.size();
    std::string current_model;
    for(int pos=0; pos < nb_models; pos++){
        current_model = object_name_vector[pos];
        geometry_msgs::Pose current_model_pose;
        if ((std::strcmp(current_model.c_str(), "cube") == 0) ||
           (std::strcmp(current_model.c_str(), "cylinder") == 0) ||
           (std::strcmp(current_model.c_str(), "bucket") == 0)) {
            current_model_pose = get_model_pose(nh, current_model);
        } else {
            ROS_ERROR_STREAM("spawn_environment method : unknown model " << current_model);
            return false;
        }
        ROS_INFO_STREAM(current_model << " " << current_model_pose);
        spawn_model(current_model,
                    spawner,
                    current_model_pose,
                    project_name);
    }
    return true;
}

/**
 * @brief the function to spawn objects
 * @param name of the object to spawn, and its pose and the service client
 * @param argv
 * @return bool that sees if it succeed in spawning the model in gazebo or not
 */
bool spawn_model(std::string& model_to_spawn,
                 ros::ServiceClient& my_spawner,
                 geometry_msgs::Pose& model_pose,
                 std::string project_name){
    gazebo_msgs::SpawnModel my_model;
    char* pPath;
    pPath = getenv ("PWD");
    std::string model_file_location;
    model_file_location.append(pPath);
    model_file_location.append("/src/" + project_name + "/world/");
    model_file_location.append(model_to_spawn);
    model_file_location.append("/model.sdf");
    //ROS_INFO_STREAM("file location is: " << model_file_location.c_str());
    std::ifstream model_file(model_file_location.c_str());
    if (model_file){
        std::string line;
        while (!model_file.eof()) {
            std::getline(model_file,line);
            my_model.request.model_xml+=line;
        }
        model_file.close();
        my_model.request.model_name = model_to_spawn;
        my_model.request.reference_frame = "base";
        my_model.request.initial_pose = model_pose;
        my_spawner.call(my_model);
        return my_model.response.success;
    } else {
        ROS_ERROR_STREAM("File " << model_file_location << " not found while spawning");
        exit(-1);
    }
}

/**
 * a
 */
geometry_msgs::Pose get_model_pose(ros::NodeHandle nh,
                                    std::string model_name){
    geometry_msgs::Pose object_pose;
    nh.getParam("/obj_pos_vector/" + model_name + "/x", object_pose.position.x);
    nh.getParam("/obj_pos_vector/" + model_name + "/y", object_pose.position.y);
    nh.getParam("/obj_pos_vector/" + model_name + "/z", object_pose.position.z);
//    if (strcmp(model_name.c_str(), "table") == 0) {
//        tf::Quaternion tmp_orientation;
//        double roll, pitch, yaw;
//        nh.getParam("/obj_orien_vector/" + model_name + "/roll", roll);
//        nh.getParam("/obj_orien_vector/" + model_name + "/pitch", pitch);
//        nh.getParam("/obj_orien_vector/" + model_name + "/yaw", yaw);
//        tmp_orientation.setRPY(roll,
//                               pitch,
//                               yaw);
//        object_pose.orientation.w = tmp_orientation.getW();
//        object_pose.orientation.x = tmp_orientation.getX();
//        object_pose.orientation.y = tmp_orientation.getY();
//        object_pose.orientation.z = tmp_orientation.getZ();

////    } else if (strcmp(model_name.c_str(), "cube") == 0) {
//    } else {
    double orien_w, orien_x, orien_y, orien_z;
    nh.getParam("/obj_orien_vector/" + model_name + "/w", orien_w);
    nh.getParam("/obj_orien_vector/" + model_name + "/x", orien_x);
    nh.getParam("/obj_orien_vector/" + model_name + "/y", orien_y);
    nh.getParam("/obj_orien_vector/" + model_name + "/z", orien_z);
    object_pose.orientation.w = orien_w;
    object_pose.orientation.x = orien_x;
    object_pose.orientation.y = orien_y;
    object_pose.orientation.z = orien_z;
//    }

    return object_pose;
}

/**
 * @brief remove the spawned models
 * @param a
 * @param a
 * @return bool that
 */
bool remove_environment(ros::ServiceClient& gazebo_model_delete,
                        ros::NodeHandle nh){
    std::vector<std::string> object_name_vector;   
    nh.getParam("obj_name_vector", object_name_vector);
    object_name_vector.push_back("table");
    int nb_models = object_name_vector.size();
    std::string current_model;
    for(int pos=0; pos < nb_models; pos++){
        current_model = object_name_vector[pos];
        remove_model(current_model, gazebo_model_delete);
    }
    return true;
}

/**
 * @brief remove one model
 * @param a
 * @param a
 * @return bool that
 */
bool remove_model(std::string model_name,
                  ros::ServiceClient& gazebo_model_delete){
    std::string current_model = model_name;
    gazebo_msgs::DeleteModel delete_model;
    if ((std::strcmp(current_model.c_str(), "table") == 0) || 
       (std::strcmp(current_model.c_str(), "cube") == 0) || 
       (std::strcmp(current_model.c_str(), "cylinder") == 0) ||
       (std::strcmp(current_model.c_str(), "bucket") == 0)) {
        delete_model.request.model_name = model_name;
    } else {
        ROS_ERROR_STREAM("remove_model method : unknown model " << current_model);
        return false;
    }
    gazebo_model_delete.call(delete_model);

    return true;
}

/**
 * @brief get spawned model values
 * @param a
 * @param a
 * @return bool that
 */
std::tuple<gazebo_msgs::GetModelState, double, double, double> get_object_values(
        std::string model_name,
        ros::ServiceClient& gazebo_model_state,
        bool robot_frame){

      typedef std::tuple<gazebo_msgs::GetModelState, double, double, double> res;
      gazebo_msgs::GetModelState model_state;
      if(robot_frame)
          model_state.request.relative_entity_name = "base";      
      model_state.request.model_name = model_name;
      gazebo_model_state.call(model_state);

      tf::Quaternion quat;
      quat.setW(model_state.response.pose.orientation.w);
      quat.setX(model_state.response.pose.orientation.x);
      quat.setY(model_state.response.pose.orientation.y);
      quat.setZ(model_state.response.pose.orientation.z);

      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      return res(model_state, roll, pitch, yaw);
}

/**
 * @brief get a link within spawned model values
 * @param a
 * @param a
 * @return bool that
 */
std::tuple<gazebo_msgs::GetLinkState, double, double, double> get_link_values(
        std::string link_name,
        ros::ServiceClient& gazebo_link_state){

      typedef std::tuple<gazebo_msgs::GetLinkState, double, double, double> res;
      gazebo_msgs::GetLinkState link_state;
      link_state.request.link_name = link_name;
      ROS_INFO_STREAM("my link name is: " << link_state.request.link_name);
      link_state.request.reference_frame = "world";
      gazebo_link_state.call(link_state);
      ROS_INFO_STREAM("my reference frame is: " << link_state.request.reference_frame);
      ROS_INFO_STREAM("pure answer: " << link_state.response);
      tf::Quaternion quat;
      quat.setW(link_state.response.link_state.pose.orientation.w);
      quat.setX(link_state.response.link_state.pose.orientation.x);
      quat.setY(link_state.response.link_state.pose.orientation.y);
      quat.setZ(link_state.response.link_state.pose.orientation.z);
      //ROS_INFO_STREAM("answer: " << link_state.response.link_state.pose.orientation << link_state.response.link_state.pose.position);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      return res(link_state, roll, pitch, yaw);
}

float RandomNumber(float Min, float Max)
{
    return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}

/**
 * @brief define coordinates of all points (object sides, and intermediate points)
 * @param a
 * @param a
 * @return bool that
 */
void compute_predefined_scenario_wps(
        std::vector<Eigen::Vector3d>& side_position_vector,
        std::vector<Eigen::Vector3d>& mid_point_position_vector,
        geometry_msgs::Pose& object_pose,
        Env_values env_values,
        bool random_values){

    double tmp_cube_side = env_values.get_cube_side();
    double tmp_mid_point_dist = env_values.get_mid_point_dist();
    double cube_z = -0.135;

    float tmp_random = 0;
    if (random_values){

        tmp_random = RandomNumber(-0.025, 0.025);

        ROS_ERROR_STREAM("rand value" << tmp_random);

        side_position_vector.clear();
        side_position_vector.push_back({object_pose.position.x + tmp_random/2,
                                 object_pose.position.y - tmp_cube_side/2.0 + tmp_random/2,
                                 cube_z + tmp_random/2}); //RIGHT
        side_position_vector.push_back({object_pose.position.x  + tmp_random/2,
                                 object_pose.position.y + tmp_cube_side/2.0 + tmp_random/2,
                                 cube_z + tmp_random/2}); //LEFT
        side_position_vector.push_back({object_pose.position.x - tmp_cube_side/2.0 + tmp_random/2,
                                 object_pose.position.y + tmp_random/2,
                                 cube_z + tmp_random/2}); //FAR
        side_position_vector.push_back({object_pose.position.x + tmp_cube_side/2.0 + tmp_random/2,
                                 object_pose.position.y + tmp_random/2,
                                 cube_z + tmp_random/2}); //CLOSE

        //Right_mid, Left_mid, Front_mid, Back_mid, Front_r, Front_l, Back_r, Back_l
        mid_point_position_vector.clear();
        mid_point_position_vector.push_back({object_pose.position.x + tmp_random,
                                             object_pose.position.y - tmp_cube_side/2.0 - tmp_mid_point_dist + tmp_random,
                                             cube_z + tmp_random}); //Right_mid
        mid_point_position_vector.push_back({object_pose.position.x + tmp_random,
                                             object_pose.position.y + tmp_cube_side/2.0 + tmp_mid_point_dist + tmp_random,
                                             cube_z + tmp_random}); //Left_mid
        mid_point_position_vector.push_back({object_pose.position.x + tmp_cube_side/2.0 + tmp_mid_point_dist + tmp_random,
                                             object_pose.position.y + tmp_random,
                                             cube_z + tmp_random}); //Far_mid
        mid_point_position_vector.push_back({object_pose.position.x - tmp_cube_side/2.0 - tmp_mid_point_dist + tmp_random,
                                             object_pose.position.y + tmp_random,
                                             cube_z + tmp_random}); //Close_mid
    }
}
