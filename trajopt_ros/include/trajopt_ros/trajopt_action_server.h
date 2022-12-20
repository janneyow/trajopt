#ifndef TRAJOPT_ACTION_SERVER_H
#define TRAJOPT_ACTION_SERVER_H

#include <ros/ros.h>
#include "trajopt_ros/GetTrajFromTrajOpt.h"

#include <new>

#include <tesseract_core/basic_types.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_msgs/Trajectory.h>

#include <urdf_parser/urdf_parser.h>

#include <trajopt/problem_description.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>


class GenTraj
{
    public:
        GenTraj();
        ~GenTraj();

    private:
        bool GenTrajCB(trajopt_ros::GetTrajFromTrajOpt::Request &req,
              trajopt_ros::GetTrajFromTrajOpt::Response &res);

        ros::NodeHandle nh, pnh;

        // service
        ros::ServiceServer service;

        // trajopt
        bool file_write_cb;

        urdf::ModelInterfaceSharedPtr urdf_model;
        srdf::ModelSharedPtr srdf_model;
        tesseract::tesseract_ros::KDLEnvPtr env;


};

// const std::string TRAJOPT_DESCRIPTION_PARAM =
//     "trajopt_description"; /**< Default ROS parameter for trajopt description */

trajopt::TrajOptProbPtr jsonMethod(tesseract::tesseract_ros::KDLEnvPtr env, std::string file_name)
{
    // ros::NodeHandle nh;
    // std::string trajopt_config;

    // nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);
    std::cout << file_name << std::endl;
    std::ifstream json_file(file_name);
    std::ostringstream tmp;
    tmp << json_file.rdbuf();
    std::string trajopt_config = tmp.str();
    
    std::cout << trajopt_config.c_str() << std::endl;

    Json::Value root;
    Json::Reader reader;
    bool parse_success = reader.parse(trajopt_config.c_str(), root);
    if (!parse_success)
    {
        ROS_FATAL("Failed to load trajopt json file from ros parameter");
    }

    return trajopt::ConstructProblem(root, env);
}

#endif // TRAJOPT_ACTION_SERVER_H