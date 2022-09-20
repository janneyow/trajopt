/**
 * @file trajopt_action_server.cpp
 * @author J-Anne (janne.yow@ntu.edu.sg)
 * @brief Service server to generate a trajectory using TrajOpt
 * @version 0.1
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <trajopt_ros/trajopt_action_server.h>

GenTraj::GenTraj() : pnh("~")
{
    service = pnh.advertiseService("get_traj_from_traj_opt", &GenTraj::GenTrajCB, this);

    nh.param<bool>("/trajopt_action_server/file_write_cb", file_write_cb, false);

    /////////////
    /// SETUP ///
    /////////////

    // Pull ROS params
    std::string urdf_xml_string, srdf_xml_string;
    nh.getParam("robot_description", urdf_xml_string);
    nh.getParam("robot_description_semantic", srdf_xml_string);

    // Initialize the environment
    urdf_model = urdf::parseURDF(urdf_xml_string);
    srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initString(*urdf_model, srdf_xml_string);
    env =  tesseract::tesseract_ros::KDLEnvPtr(new tesseract::tesseract_ros::KDLEnv);

    assert(urdf_model != nullptr);
    assert(env != nullptr);
    bool success = env->init(urdf_model, srdf_model);
    assert(success);

    ROS_INFO("[TRAJOPT_AS] Environment setup complete");
}

bool GenTraj::GenTrajCB(trajopt_ros::GetTrajFromTrajOpt::Request &req,
              trajopt_ros::GetTrajFromTrajOpt::Response &res)
{
    ROS_INFO("Received service request to generate trajectory");

    // Get initial state of the robot
    std::unordered_map<std::string, double> joint_states;
    for (int i = 0; i < req.start.name.size(); i++)
    {
        joint_states[req.start.name[i]] = double(req.start.position[i]);
        std::cout << req.start.name[i] << " " << double(req.start.position[i]) << std::endl;
    }

    std::cout << "Setting state" << std::endl;

    env->setState(joint_states);

    std::cout << "Set initial joint state" << std::endl;

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
    // config.params.max_iter = 50;

    // Create Plot Callback
    if (req.plot_traj_steps)
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
    // optimizer created in solve, config stores the json file
    planner.solve(planning_response, config);

    // TODO: return false based on planner status
    std::cout << planning_response.status_description << std::endl;


    if (file_write_cb)
        stream_ptr->close();

    // Plot the resulting trajectory
    // TODO: this doesn't plot, need to initialize the arm in rviz first
    // if (req.plot_final_traj)
    // {
    //     ROS_INFO("Plotting trajectory...");
    //     plotter.plotTrajectory(planning_response.joint_names, planning_response.trajectory.leftCols(planning_response.joint_names.size()));
    // }
    
    // Return the trajectory
    res.status = planning_response.status_code;
    tesseract_msgs::Trajectory traj = plotter.getTrajectory(planning_response.joint_names, planning_response.trajectory.leftCols(planning_response.joint_names.size()));
    res.trajectory = traj;


    ROS_INFO("RUN COMPLETE");

    return true;
}
 


int main(int argc, char **argv)
{   
    // Set log level
    util::gLogLevel = util::LevelInfo;

    ros::init(argc, argv, "get_traj_from_traj_opt_server");

    GenTraj generate_traj;

    ros::spin();

    return EXIT_SUCCESS;
}
