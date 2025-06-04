#include "types/Dataset.hh"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_dataset(py::module_ &m) {
  // Dastatset
  /// @brief A handle for accessing raw data, like the strayscanner export.
  py::class_<ReUseX::Dataset>(m, "Dataset")
      .def(py::init<const std::string &>())
      .def(py::init<const std::filesystem::path &>())
      .def("fields", &ReUseX::Dataset::fields)
      .def("intrinsic_matrix", &ReUseX::Dataset::intrinsic_matrix)
      .def("__bool__", &ReUseX::Dataset::operator bool)
      .def("__getitem__", &ReUseX::Dataset::operator[])
      .def(
          "__getitem__",
          [](ReUseX::Dataset &d, py::slice slice) {
            py::list list;
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(d.size(), &start, &stop, &step, &slicelength))
              throw py::error_already_set();

            for (size_t i = start; i < stop; i += step) {
              list.append(d[i]);
            }
            return list;
          },
          "Get a data package", "slice"_a)
      .def(
          "__getitem__",
          [](ReUseX::Dataset &d, py::list selection) {
            py::tuple result(py::len(selection));
            for (py::size_t i = 0; i < py::len(selection); ++i) {
              result[i] = d[selection[i].cast<size_t>()]; // Convert Python
                                                          // index to C++ size_t
            }
            return result;
          },
          "Get a data package", "sellection"_a)
      .def("__len__", &ReUseX::Dataset::size)
      .def_property_readonly("size", &ReUseX::Dataset::size)
      .def_property_readonly("color_size", &ReUseX::Dataset::color_size)
      .def_property_readonly("depth_size", &ReUseX::Dataset::depth_size)
      .def_property_readonly("name", &ReUseX::Dataset::name)
      .def("__repr__", [](ReUseX::Dataset const &d) -> std::string {
        return fmt::format("Dataset: {}", d.name());
      });
}
