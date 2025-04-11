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
      .value("IMU", ReUseX::Field::IMU);
  //.value("POSES", ReUseX::Field::POSES);

  /// @brief Data is the indevidual frames that the dataset provieds.
  /// Think of the data-set as a clollection of data packages.
  py::bind_map<FieldMap>(m, "DataDict", py::module_local(false));
  py::class_<ReUseX::DataItem, FieldMap>(m, "DataItem")
      .def(py::init<>())
      .def_property_readonly("color",
                             [](const ReUseX::DataItem &d) {
                               return d.get<ReUseX::Field::COLOR>();
                             })
      .def_property_readonly("depth",
                             [](const ReUseX::DataItem &d) {
                               return d.get<ReUseX::Field::DEPTH>();
                             })
      .def_property_readonly("confidence",
                             [](const ReUseX::DataItem &d) {
                               return d.get<ReUseX::Field::CONFIDENCE>();
                             })
      .def_property_readonly("odometry",
                             [](const ReUseX::DataItem &d) {
                               return d.get<ReUseX::Field::ODOMETRY>();
                             })
      .def_property_readonly(
          "imu",
          [](const ReUseX::DataItem &d) { return d.get<ReUseX::Field::IMU>(); })
      // .def_property_readonly("pose",
      //                        [](const ReUseX::DataItem &d) {
      //                          return d.get<ReUseX::Field::POSES>().matrix();
      //                        })
      .def("__repr__",
           [](ReUseX::DataItem const &d) { return fmt::format("DataItem"); })
      .def(
          "__torch_function__",
          [](py::object self, py::args args, py::kwargs kwargs) {
            return py::reinterpret_borrow<py::object>(
                py::module_::import("torch").attr("as_tensor")(py::dict(self)));
          })
      .def("fields", &ReUseX::DataItem::fields)
      .def(py::pickle(
          [](const ReUseX::DataItem &d) { //__getstate__
            std::vector<ReUseX::Field> field_keys;
            std::vector<py::object> field_values;

            for (const auto &[key, value] : d) {
              field_keys.push_back(key);

              std::visit(
                  [&](auto &&arg) {
                    field_values.push_back(
                        py::cast(arg)); // Convert value to Python
                  },
                  value);
            }

            return py::make_tuple(field_keys, field_values);
          },
          [](py::tuple t) { //__setstate__
            if (t.size() != 2) {
              throw std::runtime_error("Invalid state tuple for DataItem");
            }

            auto field_keys = t[0].cast<std::vector<ReUseX::Field>>();
            auto field_values = t[1].cast<std::vector<py::object>>();

            if (field_keys.size() != field_values.size()) {
              throw std::runtime_error("Field keys and values size mismatch");
            }

            ReUseX::DataItem d;

            for (size_t i = 0; i < field_keys.size(); ++i) {
              ReUseX::Field key = field_keys[i];

              switch (key) {
              case ReUseX::Field::INDEX:
                d[key] =
                    field_values[i]
                        .cast<ReUseX::FieldType<ReUseX::Field::INDEX>::type>();
                break;
              case ReUseX::Field::COLOR:
                d[key] =
                    field_values[i]
                        .cast<ReUseX::FieldType<ReUseX::Field::COLOR>::type>();
                break;
              case ReUseX::Field::DEPTH:
                d[key] =
                    field_values[i]
                        .cast<ReUseX::FieldType<ReUseX::Field::DEPTH>::type>();
                break;
              case ReUseX::Field::CONFIDENCE:
                d[key] = field_values[i]
                             .cast<ReUseX::FieldType<
                                 ReUseX::Field::CONFIDENCE>::type>();
                break;
              case ReUseX::Field::ODOMETRY:
                d[key] =
                    field_values[i]
                        .cast<
                            ReUseX::FieldType<ReUseX::Field::ODOMETRY>::type>();
                break;
              case ReUseX::Field::IMU:
                d[key] =
                    field_values[i]
                        .cast<ReUseX::FieldType<ReUseX::Field::IMU>::type>();
                break;
              // case ReUseX::Field::POSES:
              //   d[key] =
              //       field_values[i]
              //           .cast<ReUseX::FieldType<ReUseX::Field::POSES>::type>();
              //   break;
              default:
                throw std::runtime_error(
                    "Unknown field type encountered during deserialization");
              }
            }

            return d;

          }));
}
