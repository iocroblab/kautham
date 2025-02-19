#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

// Include other binding headers
#include "kauthamshell_bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(pykautham, m) {
    m.doc() = "Python bindings for Kautham";

    // Call functions from other files to register bindings
    register_kauthamshell(m);
}