
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robconf_bindings.h"

namespace py = pybind11;
using namespace Kautham;

void register_robconf(pybind11::module &m) {
    // RobConf class
    py::class_<RobConf>(m, "RobConf")
        .def(py::init<>())
        .def("get_se3", &RobConf::getSE3)
        .def("set_se3", py::overload_cast<>(&RobConf::setSE3))
        .def("set_se3_from_coords", py::overload_cast<std::vector<double>&>(&RobConf::setSE3))
        .def("get_rn", &RobConf::getRn)
        .def("set_rn", py::overload_cast<unsigned int>(&RobConf::setRn))
        .def("set_rn_from_coords", py::overload_cast<std::vector<double>&>(&RobConf::setRn))
        .def("get_distance", py::overload_cast<RobConf&>(&RobConf::getDistance))
        .def("get_distance_with_weights", py::overload_cast<RobConf&, RobWeight&>(&RobConf::getDistance))
        .def("interpolate", &RobConf::interpolate);
}
    
