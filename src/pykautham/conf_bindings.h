#ifndef CONF_BINDINGS_H
#define CONF_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/sampling/conf.h>

void register_conf(pybind11::module &m);

#endif