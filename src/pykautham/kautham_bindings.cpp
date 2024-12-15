
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "conf.h"
#include "se3conf.h"
#include "rnconf.h"
#include "robconf.h"
#include "robweight.h"

namespace py = pybind11;
using namespace Kautham;

PYBIND11_MODULE(kautham_bindings, m) {
    m.doc() = "Pybind11 bindings for Kautham configurations";

    py::class_<Conf>(m, "Conf")
        .def("set_coordinates", &Conf::setCoordinates)
        .def("get_distance", py::overload_cast<Conf*>(&Conf::getDistance))
        .def("get_distance_with_weights", py::overload_cast<Conf*, std::vector<KthReal>&>(&Conf::getDistance))
        .def("get_coordinates", &Conf::getCoordinates)
        .def("get_dimension", &Conf::getDim)
        .def("print", &Conf::print);

    py::class_<SE3Conf, Conf>(m, "SE3Conf")
        .def(py::init<>())
        .def("set_coordinates_from_se2", &SE3Conf::setCoordinates)
        .def("set_coordinates", py::overload_cast<std::vector<KthReal>&>(&SE3Conf::setCoordinates))
        .def("get_distance", py::overload_cast<Conf*>(&SE3Conf::getDistance))
        .def("get_distance_with_weights", py::overload_cast<Conf*, KthReal&, KthReal&>(&SE3Conf::getDistance))
        .def("interpolate", &SE3Conf::interpolate)
        .def("print", &SE3Conf::print);

    py::class_<RnConf, Conf>(m, "RnConf")
        .def(py::init<unsigned int>())
        .def("resize", &RnConf::reDim)
        .def("get_distance", py::overload_cast<Conf*>(&RnConf::getDistance))
        .def("get_distance_with_weights", py::overload_cast<Conf*, std::vector<KthReal>&>(&RnConf::getDistance))
        .def("interpolate", &RnConf::interpolate)
        .def("print", &RnConf::print);

    py::class_<RobConf>(m, "RobConf")
        .def(py::init<>())
        .def("get_se3", &RobConf::getSE3)
        .def("set_se3", py::overload_cast<>(&RobConf::setSE3))
        .def("set_se3_from_coords", py::overload_cast<std::vector<KthReal>&>(&RobConf::setSE3))
        .def("get_rn", &RobConf::getRn)
        .def("set_rn", py::overload_cast<unsigned int>(&RobConf::setRn))
        .def("set_rn_from_coords", py::overload_cast<std::vector<KthReal>&>(&RobConf::setRn))
        .def("get_distance", py::overload_cast<RobConf&>(&RobConf::getDistance))
        .def("get_distance_with_weights", py::overload_cast<RobConf&, RobWeight&>(&RobConf::getDistance))
        .def("interpolate", &RobConf::interpolate);

    py::class_<RobWeight>(m, "RobWeight")
        .def(py::init<unsigned int>())
        .def("set_rn_dimension", &RobWeight::setRnDim)
        .def("set_se3_weight", &RobWeight::setSE3Weight)
        .def("get_se3_weight", &RobWeight::getSE3Weight)
        .def("get_rn_weights", &RobWeight::getRnWeights);
}
