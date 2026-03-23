#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "franka_IK_HE.hpp"

namespace py = pybind11;

PYBIND11_MODULE(franka_ik, m) {
    m.doc() = "Franka IK Python bindings";

    m.def("franka_IK_EE",
        &franka_IK_EE,
        py::arg("O_T_EE"),
        py::arg("q7"),
        py::arg("q_actual")
    );

    m.def("franka_IK_EE_CC",
        &franka_IK_EE_CC,
        py::arg("O_T_EE"),
        py::arg("q7"),
        py::arg("q_actual")
    );
}