#include <pybind11/pybind11.h>
#include "robweight_bindings.h"

namespace py = pybind11;

void register_robweight(pybind11::module &m) {
    // RobWeight class
    py::class_<Kautham::RobWeight>(m, "RobWeight")
        .def(py::init<unsigned int>())
        .def("setRnDim", &Kautham::RobWeight::setRnDim)
        .def("getSE3Weight", &Kautham::RobWeight::getSE3Weight)
        .def("getRnWeights", &Kautham::RobWeight::getRnWeights);
}
