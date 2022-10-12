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
#include <Python.h>
#include <Eigen/Eigenvalues>
#include <string>
#include <sstream>
#include <iostream>
#include <locale>


std::wstring widen( const std::string& str )
{
    std::wostringstream wstm ;
    const std::ctype<wchar_t>& ctfacet = std::use_facet<std::ctype<wchar_t>>(wstm.getloc()) ;
    for( size_t i=0 ; i<str.size() ; ++i ) 
              wstm << ctfacet.widen( str[i] ) ;
    return wstm.str() ;
}

struct DistFromPt : public sco::ScalarOfVector {
  Eigen::VectorXd pt_;
  DistFromPt(const Eigen::VectorXd& pt) : pt_(pt) {}
  double operator()(const Eigen::VectorXd& x) const {
    // std::cout << "Cost function" << std::endl;
    std::cout << "returns: " << (x-pt_).squaredNorm() << std::endl;

    return (x-pt_).squaredNorm();
  }
};

// double table_cost(const Eigen::VectorXd& x){

// }

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

    // -------------------
       // Initialize python instance
    const auto python_path = "/home/janne/ros_ws/ferl/devel/lib/python3/dist-packages:/home/janne/phd1/xarm/devel/lib/python3/dist-packages:/home/janne/phd1/shared_control/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/janne/ros_ws/ferl/src/trajopt";
    // const auto python_path = "/home/janne/anaconda3/envs/mj38/lib/python3.8";
    // Py_SetPath(widen(python_path).c_str());
    const auto python_home = "/home/janne/anaconda3/envs/mj38";
    Py_SetPythonHome(widen(python_home).c_str());
    const auto python_program_name = "/home/janne/anaconda3/envs/mj38/bin/python3.8";
    Py_SetProgramName(widen(python_program_name).c_str());
    Py_Initialize();

    PyRun_SimpleString("import sys \nprint('SYS PATH:', sys.path)");
    PyRun_SimpleString("print('SYS version:', sys.version)");

    
    // Import python module
    PyObject* pModuleString = PyUnicode_FromString((char*)"ferl_mj.planners.trajopt_planner");
    PyObject* pModule = PyImport_Import(pModuleString);
    Py_DECREF(pModuleString);

    PyObject *pFunc, *pArgs, *pValue;
    if (pModule != NULL){
        pFunc = PyObject_GetAttrString(pModule, (char*)"test_function");

        if (pFunc && PyCallable_Check(pFunc))
        // error inside here -> to debug tmw
        {
            // const Eigen::VectorXd x(2);
            // x(0) = 0;
            // x(1) = 1;
            // std::cout << "vectorxd x:" << x << std::endl;
            pArgs = Py_BuildValue("(i)", 5); // need to pass in waypoints
            // pArgs = PyTuple_Pack(1, 5);
            std::cout << "Set pargs" << std::endl;
            // PyTuple_SetItem(pArgs, 0, )
            pValue = PyObject_CallObject(pFunc, pArgs);
            std::cout << "Called function" << std::endl;
            Py_DECREF(pArgs);

            if (pValue != NULL)
            {
                std::cout << "Result of call: " << PyLong_AsDouble(pValue);
                Py_DECREF(pValue);
            }
            else 
            {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                std::cout << "Function call failed" << std::endl;

            }

        }
        else 
        {
            if (PyErr_Occurred())
                PyErr_Print();
            std::cout << "Cannot find function" << std::endl;
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        
    }
    else 
    {
        PyErr_Print();
        std::cout << "Failed to load module" << std::endl;
    }

    auto result = pValue;
    std::cout << "result: " << result << std::endl;

}

GenTraj::~GenTraj(){
    // close python instance
    Py_Finalize();
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

    env->setState(joint_states);

    ROS_INFO("Set initial joint state");

    // Send the initial trajectory for plotting
    tesseract::tesseract_ros::ROSBasicPlotting plotter(env);
    Eigen::RowVectorXd init_pos = env->getCurrentJointValues();
    plotter.plotTrajectory(env->getJointNames(), init_pos.leftCols(env->getJointNames().size()));

    // Create planner and responses that will store the results
    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;
    tesseract::tesseract_planning::PlannerResponse planning_response_place;

    // Setup problem
    std::string file_name = req.file_name;
    trajopt::TrajOptProbPtr test_prob;
    test_prob = jsonMethod(env, file_name);

    std::cout << "test_prob->GetVars() size: " << test_prob->GetVars().size() << std::endl;
    test_prob->addCost(sco::CostPtr(new sco::CostFromFunc(sco::ScalarOfVectorPtr(new DistFromPt(Eigen::Vector2d(2,1))), test_prob->getVars(), "table_cost")));

    
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
