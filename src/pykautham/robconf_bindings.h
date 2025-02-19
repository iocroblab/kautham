#ifndef ROBCONF_BINDINGS_H
#define ROBCONF_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/sampling/robconf.h>

void register_robconf(pybind11::module &m);

#endif