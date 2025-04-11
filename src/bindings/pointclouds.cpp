#include "types/Geometry/PointClouds.hh"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_pointclouds(py::module_ &m) {

  /// @brief Collection of point clouds in memory class
  py::class_<ReUseX::PointCloudsInMemory>(m, "PointCloudsInMemory")
      .def(py::init<const std::string &>(), "Load a point cloud from disk",
           "path"_a,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def_static("load", &ReUseX::PointCloudsInMemory::load,
                  "Load a point cloud from disk", "path"_a,
                  py::call_guard<py::scoped_ostream_redirect,
                                 py::scoped_estream_redirect>())
      .def("filter", &ReUseX::PointCloudsInMemory::filter,
           "Filter the point cloud", "value"_a = 2,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("register", &ReUseX::PointCloudsInMemory::register_clouds,
           "Register the point clouds",
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("merge", &ReUseX::PointCloudsInMemory::merge,
           "Merge the point clouds",
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("hdf5", &ReUseX::PointCloudsInMemory::annotate_from_hdf5,
           "Annotate the point clouds from hdf5", "hdf5_path"_a,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def(
          "__getitem__",
          [](const ReUseX::PointCloudsInMemory &obj, std::size_t index) {
            return obj[index];
          },
          "Get a point cloud", "index"_a)
      .def(
          "__getitem__",
          [](const ReUseX::PointCloudsInMemory &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
              throw py::error_already_set();
            return obj[std::slice(start, stop, step)];
          },
          "Get a point cloud", "slice"_a)
      .def("__len__", &ReUseX::PointCloudsInMemory::size);

  /// @brief Collection of point clouds on disk class
  py::class_<ReUseX::PointCloudsOnDisk>(m, "PointCloudsOnDisk")
      .def(py::init<const std::string &>(), "Load a point cloud from disk",
           "path"_a,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def_static("load", &ReUseX::PointCloudsOnDisk::load,
                  "Load a point cloud from disk", "path"_a,
                  py::call_guard<py::scoped_ostream_redirect,
                                 py::scoped_estream_redirect>())
      .def("filter", &ReUseX::PointCloudsOnDisk::filter,
           "Filter the point cloud", "value"_a = 2,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("register", &ReUseX::PointCloudsOnDisk::register_clouds,
           "Register the point clouds",
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("merge", &ReUseX::PointCloudsOnDisk::merge, "Merge the point clouds",
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("hdf5", &ReUseX::PointCloudsOnDisk::annotate_from_hdf5,
           "Annotate the point clouds from hdf5", "hdf5_path"_a,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      // .def("annotate", &ReUseX::PointCloudsOnDisk::annotate,
      //     "Annotate the point clouds",
      //     "yolo_path"_a,
      //     "dataset"_a = py::none(),
      //     py::return_value_policy::reference_internal,
      //     py::call_guard<py::scoped_ostream_redirect,
      //     py::scoped_estream_redirect>()
      // )
      .def(
          "__getitem__",
          [](const ReUseX::PointCloudsOnDisk &obj, std::size_t index) {
            return obj[index];
          },
          "Get a point cloud", "index"_a)
      .def(
          "__getitem__",
          [](const ReUseX::PointCloudsOnDisk &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
              throw py::error_already_set();
            return obj[std::slice(start, stop, step)];
          },
          "Get a point cloud", "slice"_a)
      .def("__len__", &ReUseX::PointCloudsOnDisk::size);
}
