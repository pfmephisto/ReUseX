#include "types/Data.hh"
// #include <pybind11/eigen.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

using FieldMap = std::map<ReUseX::Field, ReUseX::FieldVariant>;
PYBIND11_MAKE_OPAQUE(FieldMap) // This is for the Data type

void bind_dataitem(py::module_ &m) {

  /// @brief The collection of data types a data package can provide.
  /// They are passed to the Dataset on construction to limmit the amount of
  /// data that will be loaded.
  py::enum_<ReUseX::Field>(m, "Field", py::arithmetic())
      .value("INDEX", ReUseX::Field::INDEX)
      .value("COLOR", ReUseX::Field::COLOR)
      .value("DEPTH", ReUseX::Field::DEPTH)
      .value("CONFIDENCE", ReUseX::Field::CONFIDENCE)
      .value("ODOMETRY", ReUseX::Field::ODOMETRY)
      .value("IMU", ReUseX::Field::IMU)
      .value("POSES", ReUseX::Field::POSES);

  /// @brief Data is the indevidual frames that the dataset provieds.
  /// Think of the data-set as a clollection of data packages.
  py::bind_map<FieldMap>(m, "DataDict", py::module_local(false));
  py::class_<ReUseX::Data, FieldMap>(m, "Data")
      .def(py::init<>())
      .def_property_readonly(
          "color",
          [](const ReUseX::Data &d) { return d.get<ReUseX::Field::COLOR>(); })
      .def_property_readonly(
          "depth",
          [](const ReUseX::Data &d) { return d.get<ReUseX::Field::DEPTH>(); })
      .def_property_readonly("confidence",
                             [](const ReUseX::Data &d) {
                               return d.get<ReUseX::Field::CONFIDENCE>();
                             })
      .def_property_readonly("odometry",
                             [](const ReUseX::Data &d) {
                               return d.get<ReUseX::Field::ODOMETRY>();
                             })
      .def_property_readonly(
          "imu",
          [](const ReUseX::Data &d) { return d.get<ReUseX::Field::IMU>(); })
      .def_property_readonly("pose",
                             [](const ReUseX::Data &d) {
                               return d.get<ReUseX::Field::POSES>().matrix();
                             })
      .def("__repr__",
           [](ReUseX::Data const &d) { return fmt::format("Data Item"); })
      .def(
          "__torch_function__",
          [](py::object self, py::args args, py::kwargs kwargs) {
            return py::reinterpret_borrow<py::object>(
                py::module_::import("torch").attr("as_tensor")(py::dict(self)));
          })
      .def("fields", &ReUseX::Data::fields);
}
