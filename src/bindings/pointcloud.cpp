#include "types/Geometry/PointCloud.hh"
#include <pybind11/iostream.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_pointcloud(py::module_ &m) {

  PYBIND11_NUMPY_DTYPE(PointT, x, y, z, normal_x, normal_y, normal_z, rgba,
                       curvature, label);

  // TODO: Capture standard out on all functions that use the progress bar
  // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/utilities.html#capturing-standard-output-from-ostream

  /// @brief PointCloud class
  py::class_<ReUseX::PointCloud>(m, "PointCloud", py::buffer_protocol())
      .def(py::init<const std::string &>(), "Load a point cloud from disk",
           "path"_a,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def(py::init([](const py::array_t<float> xyz,
                       std::optional<const py::array_t<uint8_t>> rgb,
                       std::optional<const py::array_t<float>> normal,
                       std::optional<const py::array_t<int32_t>> label) {
             /* Request a buffer descriptor from Python */
             py::buffer_info xyz_info = xyz.request();

             /* Some basic validation checks ... */
             if (xyz_info.format != py::format_descriptor<float>::format() &&
                 xyz_info.format != py::format_descriptor<double>::format())
               throw std::runtime_error(
                   "Incompatible format: expected a float or double array!");

             if (xyz_info.ndim != 2)
               throw std::runtime_error("Incompatible buffer dimension!");

             auto xyz_ = xyz.unchecked<2>();

             ReUseX::PointCloud cloud;
             (*cloud).points.resize(xyz.shape(0));
             (*cloud).height = xyz.shape(0);
             (*cloud).width = 1;

             for (size_t i = 0; i < (size_t)xyz_.shape(0); i++) {
               (*cloud).points[i].x = *xyz_.data(i, 0);
               (*cloud).points[i].y = *xyz_.data(i, 1);
               (*cloud).points[i].z = *xyz_.data(i, 2);
             }

             if (rgb.has_value()) {
               auto rgb_ = rgb.value().unchecked<2>();
               for (size_t i = 0; i < (size_t)xyz_.shape(0); i++) {
                 (*cloud).points[i].r = *rgb_.data(i, 2);
                 (*cloud).points[i].g = *rgb_.data(i, 1);
                 (*cloud).points[i].b = *rgb_.data(i, 0);
               }
             }

             if (normal.has_value()) {
               auto normal_ = normal.value().unchecked<2>();
               for (size_t i = 0; i < (size_t)xyz_.shape(0); i++) {
                 (*cloud).points[i].normal_x = *normal_.data(i, 0);
                 (*cloud).points[i].normal_y = *normal_.data(i, 1);
                 (*cloud).points[i].normal_z = *normal_.data(i, 2);
               }
             }

             if (label.has_value()) {
               auto label_ = label.value().unchecked<2>();
               for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                 (*cloud).points[i].label = *label_.data(i, 0);
             }

             return cloud;
           }),
           "Create Point Cloud from Numpy arrays", "xyz"_a,
           "rgb"_a = py::none(), "normal"_a = py::none(),
           "label"_a = py::none(),
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def_static("load", &ReUseX::PointCloud::load,
                  "Load a point cloud from disk", "path"_a,
                  py::call_guard<py::scoped_ostream_redirect,
                                 py::scoped_estream_redirect>())
      .def("save", &ReUseX::PointCloud::save, "Save a point cloud to disk",
           "output_file"_a, "binary"_a = true,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("filter", &ReUseX::PointCloud::filter, "Filter the point cloud",
           "value"_a = 2, py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("downsample", &ReUseX::PointCloud::downsample,
           "Downsample the point cloud", "leaf_size"_a = 0.02,
           py::return_value_policy::reference,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("region_growing", &ReUseX::PointCloud::region_growing,
           "Region growing",
           "angle_threshold"_a = float(0.96592583), // cos(25°)
           "plane_dist_threshold"_a = float(0.1),
           "minClusterSize"_a = int(2 * (1 / 0.02) * (1 / 0.02)),
           "early_stop"_a = float(0.3), "radius"_a = float(0.1),
           "interval_0"_a = float(16), "interval_factor"_a = float(1.5),
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("clustering", &ReUseX::PointCloud::clustering,
           "Cluster the sematic labels in to instances",
           "cluster_tolerance"_a = 0.02, "min_cluster_size"_a = 100,
           "max_cluster_size"_a = std::numeric_limits<pcl::uindex_t>::max(),
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("solidify", &ReUseX::PointCloud::solidify,
           "Solidify the point cloud", "downsample_size"_a = 5000000,
           "sx"_a = 0.4, "sy"_a = 0.4, "expand_factor"_a = 2,
           "inflate_factor"_a = 1.2, "max_loop"_a = 10.0, "mult_factor"_a = 1.0,
           "fitting"_a = 0.20, "coverage"_a = 0.10, "complexity"_a = 0.70,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("display", &ReUseX::PointCloud::display, "Display the point cloud",
           "name"_a = "Cloud", py::return_value_policy::reference_internal)
      .def("bbox", &ReUseX::PointCloud::get_bbox,
           "Get the bounding box of the point cloud")
      .def("__len__", [](ReUseX::PointCloud &cloud) { return cloud->size(); })
      .def_buffer([](ReUseX::PointCloud &cloud) -> py::buffer_info {
        return py::buffer_info(
            cloud->points.data(), sizeof(ReUseX::PointCloud::Cloud::PointType),
            py::format_descriptor<
                ReUseX::PointCloud::Cloud::PointType>::format(),
            1, {cloud->points.size()},
            {sizeof(ReUseX::PointCloud::Cloud::PointType)});
      });
}
