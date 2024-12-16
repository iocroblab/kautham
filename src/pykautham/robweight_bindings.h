#ifndef ROBWEIGHT_BINDINGS_H
#define ROBWEIGHT_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/sampling/robweight.h>

void register_robweight(pybind11::module &m);

#endif