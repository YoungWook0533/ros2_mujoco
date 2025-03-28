#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <Eigen/Dense>

#include "robot_data.h"
#include "controller.h"

namespace bp = boost::python;
using namespace Eigen;
using namespace FR3Controller;

// Converter for std::vector<std::string>
struct VectorString_to_python
{
    static PyObject* convert(const std::vector<std::string>& vec)
    {
        boost::python::list py_list;
        for (const auto& str : vec)
        {
            py_list.append(str);
        }   
        return bp::incref(py_list.ptr());
    }
};

struct VectorString_from_python
{
    VectorString_from_python()
    {
        bp::converter::registry::push_back(&convertible, &construct, boost::python::type_id<std::vector<std::string>>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PySequence_Check(obj_ptr)) return nullptr;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, bp::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((bp::converter::rvalue_from_python_storage<std::vector<std::string>>*)data)->storage.bytes;
        new (storage) std::vector<std::string>();
        std::vector<std::string>& vec = *(std::vector<std::string>*)(storage);

        int len = PySequence_Size(obj_ptr);
        if (len < 0) bp::throw_error_already_set();
        vec.reserve(len);

        for (int i = 0; i < len; ++i)
        {
            vec.push_back(bp::extract<std::string>(PySequence_GetItem(obj_ptr, i)));
        }

        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(fr3_controller_wrapper_cpp) 
{
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, 1>>();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 4, 4>>();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>();

    bp::to_python_converter<std::vector<std::string>, VectorString_to_python>();
    VectorString_from_python();

    // Bind RobotData class
    bp::class_<RobotData, boost::noncopyable>("RobotData", bp::init<std::string>())
        .def("updateState", &RobotData::updateState)
        .def("getJointNames", &RobotData::getJointNames)
        .def("computePose", &RobotData::computePose)
        .def("getPose", &RobotData::getPose)
        .def("computeJacobian", &RobotData::computeJacobian)
        .def("getJacobian", &RobotData::getJacobian)
        .def("computeJacobianTimeVariation", &RobotData::computeJacobianTimeVariation)
        .def("getJacobianTimeVariation", &RobotData::getJacobianTimeVariation)
        .def("computeVelocity", &RobotData::computeVelocity)
        .def("getVelocity", &RobotData::getVelocity)
        .def("computeMassMatrix", &RobotData::computeMassMatrix)
        .def("getMassMatrix", &RobotData::getMassMatrix)
        .def("computeCoriolis", &RobotData::computeCoriolis)
        .def("getCoriolis", &RobotData::getCoriolis)
        .def("computeGravity", &RobotData::computeGravity)
        .def("getGravity", &RobotData::getGravity)
        .def("computeNonlinearEffects", &RobotData::computeNonlinearEffects)
        .def("getNonlinearEffects", &RobotData::getNonlinearEffects)
        .def("computeTaskMassMatrix", &RobotData::computeTaskMassMatrix)
        .def("getTaskMassMatrix", &RobotData::getTaskMassMatrix)
        .def("computeTaskCoriolis", &RobotData::computeTaskCoriolis)
        .def("getTaskCoriolis", &RobotData::getTaskCoriolis)
        .def("computeTaskGravity", &RobotData::computeTaskGravity)
        .def("getTaskGravity", &RobotData::getTaskGravity)
        .def("computeTaskNonlinearEffects", &RobotData::computeTaskNonlinearEffects)
        .def("getTaskNonlinearEffects", &RobotData::getTaskNonlinearEffects)
        ;

    // Bind Controller class
    bp::class_<Controller, boost::noncopyable>("Controller", bp::init<double>())
        .def("tmpControl", &Controller::tmpControl)
        ;  
}
