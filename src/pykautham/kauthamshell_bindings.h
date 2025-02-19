#ifndef KAUTHAMSHELL_BINDINGS_H
#define KAUTHAMSHELL_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/kauthamshell.h>

void register_kauthamshell(pybind11::module &m);

#endif