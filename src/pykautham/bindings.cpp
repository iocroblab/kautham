
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "conf.h"
#include "se3conf.h"
#include "rnconf.h"
#include "robconf.h"
#include "robweight.h"

namespace py = pybind11;

PYBIND11_MODULE(kautham_bindings, m) {
    m.doc() = "Python bindings for Kautham configurations";

    // Conf class
    py::class_<Kautham::Conf>(m, "Conf")
        .def(py::init<Kautham::CONFIGTYPE>())
        .def("setCoordinates", &Kautham::Conf::setCoordinates)
        .def("getDistance", py::overload_cast<Kautham::Conf*>(&Kautham::Conf::getDistance))
        .def("getDistance2", &Kautham::Conf::getDistance2)
        .def("getCoordinates", &Kautham::Conf::getCoordinates)
        .def("getDim", &Kautham::Conf::getDim);

    // SE3Conf class
    py::class_<Kautham::SE3Conf, Kautham::Conf>(m, "SE3Conf")
        .def(py::init<>())
        .def("setCoordinates", py::overload_cast<Kautham::SE2Conf*>(&Kautham::SE3Conf::setCoordinates))
        .def("getDistance2", py::overload_cast<Kautham::Conf*>(&Kautham::SE3Conf::getDistance2))
        .def("getPos", &Kautham::SE3Conf::getPos)
        .def("setPos", &Kautham::SE3Conf::setPos);

    // RnConf class
    py::class_<Kautham::RnConf, Kautham::Conf>(m, "RnConf")
        .def(py::init<unsigned int>())
        .def("reDim", &Kautham::RnConf::reDim)
        .def("getDistance2", py::overload_cast<Kautham::Conf*>(&Kautham::RnConf::getDistance2));

    // RobConf class
    py::class_<Kautham::RobConf>(m, "RobConf")
        .def(py::init<>())
        .def("getSE3", &Kautham::RobConf::getSE3)
        .def("setSE3", py::overload_cast<>(&Kautham::RobConf::setSE3))
        .def("getRn", &Kautham::RobConf::getRn)
        .def("setRn", py::overload_cast<unsigned int>(&Kautham::RobConf::setRn));

    // RobWeight class
    py::class_<Kautham::RobWeight>(m, "RobWeight")
        .def(py::init<unsigned int>())
        .def("setRnDim", &Kautham::RobWeight::setRnDim)
        .def("getSE3Weight", &Kautham::RobWeight::getSE3Weight)
        .def("getRnWeights", &Kautham::RobWeight::getRnWeights);
}
