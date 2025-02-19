#ifndef SE3CONF_BINDINGS_H
#define SE3CONF_BINDINGS_H

#include <pybind11/pybind11.h>
#include <kautham/sampling/se3conf.h>

void register_se3conf(pybind11::module &m);

#endif