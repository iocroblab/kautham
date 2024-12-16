#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robconf_bindings.h"


namespace py = pybind11;

void register_rnconf(pybind11::module &m){
    // RnConf class
    py::class_<Kautham::RnConf, Kautham::Conf>(m, "RnConf")
        .def(py::init<unsigned int>())
        .def("reDim", &Kautham::RnConf::reDim)
        .def("getDistance2", py::overload_cast<Kautham::Conf*>(&Kautham::RnConf::getDistance2));
}
