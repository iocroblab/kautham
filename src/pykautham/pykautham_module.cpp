#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

// Include other binding headers
#include "kauthamshell_bindings.h"
// #include "robconf_bindings.h"
// #include "se3conf_bindings.h"
// #include "conf_bindings.h"
#include "rnconf_bindings.h"
#include "robweight_bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(pykautham, m) {
    m.doc() = "Python bindings for Kautham";

    // Call functions from other files to register bindings
    register_kauthamshell(m);
    //register_robconf(m);
    //register_se3conf(m);
    //register_conf(m);
    register_rnconf(m);
    register_robweight(m);
}