#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

// Maybe this can be removed in release builds
#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;

void bind_functions(py::module_ &);
void bind_types(py::module_ &);

PYBIND11_MODULE(_core, m) {

  m.doc() = R"pbdoc(
        ReUseX - Reuse Explorer
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";

  bind_types(m);
  bind_functions(m);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
