
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "se3conf_bindings.h"

namespace py = pybind11;

void register_se3conf(pybind11::module &m) {
    
    py::class_<Kautham::SE3Conf>(m, "SE3Conf")
        .def(py::init<>())

        // Setters and getters
        .def("set_coordinates", 
            py::overload_cast<std::vector<Kautham::double>&>(&Kautham::SE3Conf::setCoordinates), 
            py::arg("coordinates"))
        .def("get_pos", &Kautham::SE3Conf::getPos)
        .def("set_pos", &Kautham::SE3Conf::setPos, py::arg("pos"))
        .def("get_orient", &Kautham::SE3Conf::getOrient)
        .def("set_orient", &Kautham::SE3Conf::setOrient, py::arg("orient"))
        .def("get_angle", &Kautham::SE3Conf::getAngle)
        .def("set_angle", &Kautham::SE3Conf::setAngle, py::arg("angle"))

        // Distance calculations
        .def("get_distance2", 
            py::overload_cast<const Kautham::Conf*>(&Kautham::SE3Conf::getDistance2, &Kautham::SE3Conf::getDistance2),
            py::arg("conf"))
        .def("get_distance2_weighted", 
            py::overload_cast<const Kautham::Conf*, Kautham::double, Kautham::double>(&Kautham::SE3Conf::getDistance2),
            py::arg("conf"), py::arg("trans_weight"), py::arg("rot_weight"))

        // Utility methods
        .def("print", &Kautham::SE3Conf::print)
    ;
}
