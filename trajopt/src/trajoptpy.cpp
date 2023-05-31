#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/modeling_utils.hpp>
// #include <trajopt_ros/trajopt/collision_checker.hpp>
// #include <tesseract_core/basic_kin.h>
#include <trajopt/collision_terms.hpp>
#include <trajopt/numpy_utils.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/macros.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>


#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

using namespace trajopt;
using namespace Eigen;
using std::vector;

namespace py = boost::python;

const tesseract::BasicPlottingPtr gInteractive = nullptr;

// tesseract uses KDLEnvPtr under kdl_env.h
// EnvironmentBasePtr GetCppEnv(py::object py_env)
// {
//   py::object openravepy = py::import("openravepy");
//   int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
//   EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
//   return cpp_env;
// }
// KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env)
// {
//   int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
//   return env->GetBodyFromEnvironmentId(id);
// }
// KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env)
// {
//   KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
//   int idx = py::extract<int>(py_link.attr("GetIndex")());
//   return parent->GetLinks()[idx];
// }

class PyTrajOptProb
{
public:
    TrajOptProbPtr m_prob;
    PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob){}
    // py::list GetDOFIndices()
    // {
    //   tesseract::BasicKinConstPtr kin = boost::dynamic_pointer_cast<tesseract::BasicKinConstPtr>(m_prob->GetKin());
    //   if (!kin)
    //     PRINT_AND_THROW("can only call GetDOFIndices on a robot");
    //   vector<int> inds = rad->GetJointIndices();
    //   return toPyList(inds);
    // }
    // void SetRobotActiveDOFs()
    // {
    //   RobotAndDOFPtr rad = boost::dynamic_pointer_cast<RobotAndDOF>(m_prob->GetRAD());
    //   if (!rad)
    //     PRINT_AND_THROW("can only call SetRobotActiveDOFs on a robot");
    //   rad->SetRobotActiveDOFs();
    // }
    
    void AddConstraint1(py::object f, py::list ijs, const std::string& typestr, const std::string& name);
    void AddConstraint2(py::object f, py::object dfdx, py::list ijs, const std::string& typestr, const std::string& name);
    void AddCost1(py::object f, py::list ijs, const std::string& name);
    void AddErrCost1(py::object f, py::list ijs, const std::string& typestr, const std::string& name);
    void AddErrCost2(py::object f, py::object dfdx, py::list ijs, const std::string& typestr, const std::string& name);
};

char const* greet(){
    return "Hello, world";
}

struct ScalarFuncFromPy : public sco::ScalarOfVector
{
    py::object m_pyfunc;
    ScalarFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
    double operator()(const VectorXd& x) const
    {
        return py::extract<double>(m_pyfunc(toNdarray1<double>(x.data(), x.size())));
    }
};
struct VectorFuncFromPy : public sco::VectorOfVector
{
    py::object m_pyfunc;
    VectorFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
    VectorXd operator()(const VectorXd& x) const
    {
        py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())), "float64");
        VectorXd out = Map<const VectorXd>(getPointer<double>(outarr), py::extract<int>(outarr.attr("size")));
        return out;
    }
};
struct MatrixFuncFromPy : public sco::MatrixOfVector
{
    py::object m_pyfunc;
    MatrixFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
    MatrixXd operator()(const VectorXd& x) const
    {
        py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())), "float64");
        py::object shape = outarr.attr("shape");
        MatrixXd out =
            Map<const MatrixXd>(getPointer<double>(outarr), py::extract<int>(shape[0]), py::extract<int>(shape[1]));
        return out;
    }
};

sco::ConstraintType _GetConstraintType(const std::string& typestr)
{
    if (typestr == "EQ")
       return sco::EQ;
    else if (typestr == "INEQ")
        return sco::INEQ;
    else
        PRINT_AND_THROW("type must be \"EQ\" or \"INEQ\"");
}
sco::PenaltyType _GetPenaltyType(const std::string& typestr)
{
    if (typestr == "SQUARED")
        return sco::SQUARED;
    else if (typestr == "ABS")
        return sco::ABS;
    else if (typestr == "HINGE")
        return sco::HINGE;
    else
        PRINT_AND_THROW("type must be \"SQUARED\" or \"ABS\" or \"HINGE\"r");
}
sco::VarVector _GetVars(py::list ijs, const VarArray& vars)
{
    sco::VarVector out;
    int n = py::len(ijs);
    for (int k = 0; k < n; ++k)
    {
        int i = py::extract<int>(ijs[k][0]);
        int j = py::extract<int>(ijs[k][1]);
        out.push_back(vars(i, j));
    }
    return out;
}

void PyTrajOptProb::AddConstraint1(py::object f, py::list ijs, const std::string& typestr, const std::string& name)
{
    sco::ConstraintType type = _GetConstraintType(typestr);
    sco::VarVector vars = _GetVars(ijs, m_prob->GetVars());
    sco::ConstraintPtr c(
        new TrajOptConstraintFromErrFunc(sco::VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd::Ones(0), type, name));
    m_prob->addConstraint(c);
}
void PyTrajOptProb::AddConstraint2(py::object f,
                                  py::object dfdx,
                                  py::list ijs,
                                  const std::string& typestr,
                                  const std::string& name)
{
    sco::ConstraintType type = _GetConstraintType(typestr);
    sco::VarVector vars = _GetVars(ijs, m_prob->GetVars());
    sco::ConstraintPtr c(new TrajOptConstraintFromErrFunc(sco::VectorOfVectorPtr(new VectorFuncFromPy(f)),
                                          sco::MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)),
                                          vars,
                                          VectorXd::Ones(0),
                                          type,
                                          name));
    m_prob->addConstraint(c);
}
void PyTrajOptProb::AddCost1(py::object f, py::list ijs, const std::string& name)
{
  sco::VarVector vars = _GetVars(ijs, m_prob->GetVars());
  sco::CostPtr c(new sco::CostFromFunc(sco::ScalarOfVectorPtr(new ScalarFuncFromPy(f)), vars, name));
  m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost1(py::object f, py::list ijs, const std::string& typestr, const std::string& name)
{
    sco::PenaltyType type = _GetPenaltyType(typestr);
    sco::VarVector vars = _GetVars(ijs, m_prob->GetVars());
    sco::CostPtr c(new TrajOptCostFromErrFunc(sco::VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd(), type, name));
    m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost2(py::object f, py::object dfdx, py::list ijs, const std::string& typestr, const std::string& name)
{
    sco::PenaltyType type = _GetPenaltyType(typestr);
    sco::VarVector vars = _GetVars(ijs, m_prob->GetVars());
    sco::CostPtr c(new TrajOptCostFromErrFunc(sco::VectorOfVectorPtr(new VectorFuncFromPy(f)),
                                  sco::MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)),
                                  vars,
                                  VectorXd(),
                                  type,
                                  name));
    m_prob->addCost(c);
}

Json::Value readJsonFile(const std::string& doc)
{
    Json::Value root;
    Json::Reader reader;
    bool success = reader.parse(doc, root);
    if (!success)
        throw std::invalid_argument("couldn't parse string as json");
    return root;
}

PyTrajOptProb PyConstructProblem(const std::string& json_string, py::list pos, py::list joint_names)
{    
    // Initialize the environment
        // Initialize the environment
    std::string pkg_path = ros::package::getPath("trajopt_ros");

    std::string urdf_file_path = pkg_path + "/assets/xarm6/xarm6_with_gripper.urdf";
    std::string srdf_file_path = pkg_path + "/assets/xarm6/xarm6_with_gripper.srdf";
    //
    std::cout << "URDF FILE PATH: " << urdf_file_path << std::endl;

    // for env setup, to write python class to replace this
    urdf::ModelInterfaceSharedPtr urdf_model;
    urdf_model = urdf::parseURDFFile(urdf_file_path);
    srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initFile(*urdf_model, srdf_file_path);
    tesseract::tesseract_ros::KDLEnvPtr env =  tesseract::tesseract_ros::KDLEnvPtr(new tesseract::tesseract_ros::KDLEnv);

    assert(urdf_model != nullptr);
    assert(env != nullptr);
    bool success = env->init(urdf_model, srdf_model);
    assert(success);

    // Set initial joint states
    std::unordered_map<std::string, double> joint_states;
    for (int i = 0; i < len(joint_names); i++)
    {
        std::string name = py::extract<std::string>(joint_names[i]);
        joint_states[name] = py::extract<double>(pos[i]);
    }

    env->setState(joint_states);

    Json::Value json_root = readJsonFile(json_string);
    TrajOptProbPtr cpp_prob = ConstructProblem(json_root, env);
    return PyTrajOptProb(cpp_prob);
}

// not used
const std::string GetJsonFile(const std::string file_name){
    std::cout << file_name << std::endl;
    std::ifstream json_file(file_name);
    std::ostringstream tmp;
    tmp << json_file.rdbuf();
    std::string trajopt_config = tmp.str();
    
    std::cout << trajopt_config.c_str() << std::endl;

    return trajopt_config;
}

class PyTrajOptResult
{
public:
    PyTrajOptResult(TrajOptResultPtr result) : m_result(result) {}
    TrajOptResultPtr m_result;
    py::object GetCosts()
    {
        py::list out;
        int n_costs = m_result->cost_names.size();
        for (int i = 0; i < n_costs; ++i)
        {
        out.append(py::make_tuple(m_result->cost_names[i], m_result->cost_vals[i]));
        }
        return out;
    }
    py::object GetConstraints()
    {
        py::list out;
        int n_cnts = m_result->cnt_names.size();
        for (int i = 0; i < n_cnts; ++i)
        {
        out.append(py::make_tuple(m_result->cnt_names[i], m_result->cnt_viols[i]));
        }
        return out;
    }
    py::object GetTraj()
    {
        TrajArray& traj = m_result->traj;
        py::object out = np_mod.attr("empty")(py::make_tuple(traj.rows(), traj.cols()));
        for (int i = 0; i < traj.rows(); ++i)
        {
        for (int j = 0; j < traj.cols(); ++j)
        {
            out[i][j] = traj(i, j);
        }
        }
        return out;
    }
    py::object GetStatus()
    {
        py::list out;
        out.append(m_result->status);
        return out;
    }
    py::object __str__() { return GetStatus().attr("__str__")() + GetCosts().attr("__str__")() + GetConstraints().attr("__str__")(); }
};

PyTrajOptResult PyOptimizeProblem(PyTrajOptProb& prob) 
{ 
  return OptimizeProblem(prob.m_prob, gInteractive); 
}

BOOST_PYTHON_MODULE(ctrajoptpy)
{
    np_mod = py::import("numpy");

    py::class_<PyTrajOptProb>("TrajOptProb", py::no_init)
        .def("AddConstraint",
            &PyTrajOptProb::AddConstraint1,
            "Add constraint from python function (using numerical "
            "differentiation)",
            (py::arg("f"), "var_ijs", "constraint_type", "name"))
        .def("AddConstraint",
            &PyTrajOptProb::AddConstraint2,
            "Add constraint from python error function and analytic derivative",
            (py::arg("f"), "dfdx", "var_ijs", "constraint_type", "name"))
        .def("AddCost",
            &PyTrajOptProb::AddCost1,
            "Add cost from python "
            "scalar-valued function (using "
            "numerical differentiation)",
            (py::arg("func"), "var_ijs", "name"))
        .def("AddErrorCost",
            &PyTrajOptProb::AddErrCost1,
            "Add error cost from python vector-valued error function (using "
            "numerical differentiation)",
            (py::arg("f"), "var_ijs", "penalty_type", "name"))
        .def("AddErrorCost",
            &PyTrajOptProb::AddErrCost2,
            "Add error cost from python vector-valued error function and "
            "analytic derivative",
            (py::arg("f"), "dfdx", "var_ijs", "penalty_type", "name"));

    py::def("ConstructProblem", &PyConstructProblem, "create problem from JSON string");
    py::def("OptimizeProblem", &PyOptimizeProblem);

    py::class_<PyTrajOptResult>("TrajOptResult", py::no_init)
        .def("GetCosts", &PyTrajOptResult::GetCosts)
        .def("GetConstraints", &PyTrajOptResult::GetConstraints)
        .def("GetTraj", &PyTrajOptResult::GetTraj)
        .def("GetStatus", &PyTrajOptResult::GetStatus)
        .def("__str__", &PyTrajOptResult::__str__);

    py::def("greet", greet);
    py::def("get_json_file", GetJsonFile);
  // py::class_<PyCollisionChecker>("CollisionChecker", py::no_init)
  //     .def("AllVsAll", &PyCollisionChecker::AllVsAll)
  //     .def("BodyVsAll", &PyCollisionChecker::BodyVsAll)
  //     .def("PlotCollisionGeometry", &PyCollisionChecker::PlotCollisionGeometry)
  //     .def("ExcludeCollisionPair", &PyCollisionChecker::ExcludeCollisionPair)
  //     .def("IncludeCollisionPair", &PyCollisionChecker::IncludeCollisionPair);
  // py::def("GetCollisionChecker", &PyGetCollisionChecker);
  // py::class_<PyCollision>("Collision", py::no_init).def("GetDistance", &PyCollision::GetDistance);
  // py::class_<PyGraphHandle>("GraphHandle", py::no_init).def("SetTransparency", &PyGraphHandle::SetTransparency1);

  // py::class_<PyOSGViewer>("OSGViewer", py::no_init)
  //     .def("UpdateSceneData", &PyOSGViewer::UpdateSceneData)
  //     .def("Step", &PyOSGViewer::Step)
  //     .def("PlotKinBody", &PyOSGViewer::PlotKinBody)
  //     .def("PlotLink", &PyOSGViewer::PlotLink)
  //     .def("SetTransparency", &PyOSGViewer::SetTransparency)
  //     .def("SetAllTransparency", &PyOSGViewer::SetAllTransparency)
  //     .def("Idle", &PyOSGViewer::Idle)
  //     .def("DrawText", &PyOSGViewer::DrawText);
  // py::def("GetViewer", &PyGetViewer, "Get OSG viewer for environment or create a new one");
}
