#include "visualizer/visualizer.hh"

#include <fmt/core.h>
#include <memory>
#include <optional>
#include <pybind11/detail/common.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl/filesystem.h>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include <pybind11/stl.h>

#include <spdlog/spdlog.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace std::literals::chrono_literals;

void bind_visualization(py::module_ &m) {

  py::class_<ReUseX::Visualizer,
             std::unique_ptr<ReUseX::Visualizer, py::nodelete>>(m, "Viewer",
                                                                "Viewer")
      //.def(py::init(&ReUseX::Visualizer::instance),
      //     py::return_value_policy::reference)
      //.def(py::init([]() {
      //  return std::unique_ptr<ReUseX::Visualizer, py::nodelete>(
      //      ReUseX::Visualizer::getInstance());
      //}))
      .def("__str__",
           [](ReUseX::Visualizer &) { return fmt::format("Viewer"); });

  py::class_<py::object>(m, "Visualizer", "Visualizer Context Manager")
      .def(py::init())
      .def_static(
          "__enter__",
          [](/*ReUseX::Visualizer &*/)
          /*-> std::unique_ptr<ReUseX::Visualizer, py::nodelete> */ {
            spdlog::trace("__enter__ Visualizer context manager");
            return std::unique_ptr<ReUseX::Visualizer, py::nodelete>(
                ReUseX::Visualizer::getInstance());
          },
          py::return_value_policy::reference,
          "Enter Visualization Context, creates the viewer context and makes "
          "it avalible to funtictions")
      .def_static(
          "__exit__",
          [](/*ReUseX::Visualizer &  self,*/
             const std::optional<pybind11::type> & /*exc_type*/,
             const std::optional<pybind11::object> & /*exc_value*/,
             const std::optional<pybind11::object> & /*traceback*/) {
            spdlog::trace("__exit__ Visualizer context manager");
            ReUseX::Visualizer::resetViewer();
          },
          "Exit visualizer, closes the Window and frees up recources")
      .def("__str__", [](py::object &) {
        return fmt::format("ReUseX Viewer Context Manager");
      });

  // py::module_::import("atexit").attr("register")(py::cpp_function([]() {
  //   // perform cleanup here -- this function is called with the GIL held
  //   // Flush all the loggers before shutdown
  //   ReUseX::Visualizer::resetViewer();
  // }));
}
