#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "conf_bindings.h"

namespace py = pybind11;

void register_conf(pybind11::module &m){
    // Conf class
    py::class_<Kautham::Conf>(m, "Conf")
        .def(py::init<Kautham::CONFIGTYPE>())
        .def("setCoordinates", &Kautham::Conf::setCoordinates)
        .def("getDistance", py::overload_cast<Kautham::Conf*>(&Kautham::Conf::getDistance))
        .def("getDistance2", &Kautham::Conf::getDistance2)
        .def("getCoordinates", &Kautham::Conf::getCoordinates)
        .def("getDim", &Kautham::Conf::getDim);
}
