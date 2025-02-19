#ifndef RNCONF_BINDINGS_H
#define RNCONF_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/sampling/rnconf.h>

void register_rnconf(pybind11::module &m);

#endif