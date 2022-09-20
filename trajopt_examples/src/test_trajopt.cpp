#include <tesseract_core/basic_types.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>

#include <urdf_parser/urdf_parser.h>

#include <trajopt/problem_description.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>

const std::string TRAJOPT_DESCRIPTION_PARAM =
    "trajopt_description"; /**< Default ROS parameter for trajopt description */


trajopt::TrajOptProbPtr jsonMethod(tesseract::tesseract_ros::KDLEnvPtr env)
{
    ros::NodeHandle nh;
    std::string trajopt_config;

    nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);

    Json::Value root;
    Json::Reader reader;
    bool parse_success = reader.parse(trajopt_config.c_str(), root);
    if (!parse_success)
    {
        ROS_FATAL("Failed to load trajopt json file from ros parameter");
    }

    return trajopt::ConstructProblem(root, env);
}

int main(int argc, char** argv)
{
    //////////////////////
    /// INITIALIZATION ///
    //////////////////////

    ros::init(argc, argv, "test_trajopt");
    ros::NodeHandle nh;

    bool plotting_cb, file_write_cb;
    nh.param<bool>("/test_trajopt_node/plotting", plotting_cb, false);
    nh.param<bool>("/test_trajopt_node/file_write_cb", file_write_cb, false);

    // Set Log Level
    util::gLogLevel = util::LevelInfo;

    /////////////
    /// SETUP ///
    /////////////

    // Pull ROS params
    std::string urdf_xml_string, srdf_xml_string;
    nh.getParam("robot_description", urdf_xml_string);
    nh.getParam("robot_description_semantic", srdf_xml_string);

    // Initialize the environment
    // TODO: syncing this with ferl_mj
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
    srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initString(*urdf_model, srdf_xml_string);
    tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);

    assert(urdf_model != nullptr);
    assert(env != nullptr);
    bool success = env->init(urdf_model, srdf_model);
    assert(success);

    // Get the initial state of the robot -> get from ferl_mj
    std::unordered_map<std::string, double> joint_states;
    joint_states["joint1"] = 0.0;
    joint_states["joint2"] = 0.0;
    joint_states["joint3"] = 0.0;
    joint_states["joint4"] = 0.0;
    joint_states["joint5"] = 0.0;
    joint_states["joint6"] = 0.0;
    env->setState(joint_states);

    // Send the initial trajectory for plotting
    tesseract::tesseract_ros::ROSBasicPlotting plotter(env);
    Eigen::RowVectorXd init_pos = env->getCurrentJointValues();
    plotter.plotTrajectory(env->getJointNames(), init_pos.leftCols(env->getJointNames().size()));

    // Create planner and responses that will store the results
    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;
    tesseract::tesseract_planning::PlannerResponse planning_response_place;

    // Setup problem
    trajopt::TrajOptProbPtr test_prob;
    test_prob = jsonMethod(env);

    // Set the optimization parameters (Most are being left as defaults)
    tesseract::tesseract_planning::TrajOptPlannerConfig config(test_prob);
    config.params.max_iter = 50;

    // Create Plot Callback
    if (plotting_cb)
    {
        tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(new tesseract::tesseract_ros::ROSBasicPlotting(env));
        config.callbacks.push_back(PlotCallback(*test_prob, plotter_ptr));
    }

    // Create file write callback discarding any of the file's current contents
    std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
    if (file_write_cb)
    {
        std::string path = ros::package::getPath("trajopt_examples") + "/file_output_pick.csv";
        stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
        config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, test_prob));
    }

    // Solve problem. Results are stored in the response
    ROS_INFO("Solving problem...");
    planner.solve(planning_response, config);
    // optimizer created in solve, config stores the json file

    if (file_write_cb)
        stream_ptr->close();

    // std::cout << "Env joint names: " << std::endl;
    // std::vector<std::string> joint_names = env->getJointNames();
    // for (int i = 0; i < joint_names.size(); i++){
    //     std::cout << joint_names[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Traj joint names: " << std::endl;
    // joint_names = planning_response.joint_names;
    // for (int i = 0; i < joint_names.size(); i++){
    //     std::cout << joint_names[i] << " ";
    // }
    // std::cout << std::endl;
    
    // Plot the resulting trajectory
    ROS_INFO("Plotting trajectory...");
    plotter.plotTrajectory(planning_response.joint_names, planning_response.trajectory.leftCols(planning_response.joint_names.size()));

    ROS_INFO("RUN COMPLETE");
    ros::spin();
}
