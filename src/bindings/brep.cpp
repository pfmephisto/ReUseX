#include "types/Geometry/PointClouds.hh"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_brep(py::module_ &m) {

  py::class_<ReUseX::Brep>(m, "Brep")
      .def(py::init<const ReUseX::Mesh &>())
      .def("save", &ReUseX::Brep::save, "Save a Brep to disk", "filename"_a,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def_static("load", &ReUseX::Brep::load, "Load a Brep from disk",
                  "filename"_a,
                  py::call_guard<py::scoped_ostream_redirect,
                                 py::scoped_estream_redirect>())
      .def(
          "display",
          [](ReUseX::Brep &brep, std::string name = "Brep", bool show = true) {
            brep.display(name, show);
          },
          "Display a Brep")
      .def("__repr__",
           [](const ReUseX::Brep &b) {
             std::stringstream ss;
             ss << "ReUseXBrep";
             return ss.str();
           })
      .def("toRhino", [](const ReUseX::Brep &b) {
        py::object rhinolib = py::module_::import("rhino3dm");
        py::object rhino_mesh = rhinolib.attr("Brep").attr("TryConvertBrep")(b);
        py::object rhino_brep = rhinolib.attr("Brep").attr("TryConvertBrep")(b);
        return rhino_brep;
      });

  m.def("load_brep", &ReUseX::Brep::load, "Load a Brep from disk", "filename"_a,
        py::call_guard<py::scoped_ostream_redirect,
                       py::scoped_estream_redirect>());
}
