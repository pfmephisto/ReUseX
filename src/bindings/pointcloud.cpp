#include "types/Geometry/PointCloud.hh"
#include "core/io.hh"

#include <filesystem>
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/point_struct_traits.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename PointT, typename OutType, int D>
using EigenMapTypeConst =
    Eigen::Map<const Eigen::Matrix<OutType, Eigen::Dynamic, D>, Eigen::Aligned,
               Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;

template <typename PointT, typename Field, int D, typename OutType>
EigenMapTypeConst<PointT, OutType, D>
getMap(const pcl::PointCloud<PointT> &cloud) {

  constexpr size_t offset = pcl::traits::offset<PointT, Field>::value;
  constexpr size_t stride = sizeof(PointT) / sizeof(OutType);

  return EigenMapTypeConst<PointT, OutType, D>(
      reinterpret_cast<OutType *>(const_cast<PointT *>(&cloud.points[0])) +
          offset,
      cloud.size(), D,
      Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(1, stride));
}

template <typename PointT, typename Field, int D>
EigenMapTypeConst<PointT, typename pcl::traits::datatype<PointT, Field>::type,
                  D>
getMap(const pcl::PointCloud<PointT> &cloud) {
  using OutType = typename pcl::traits::datatype<PointT, Field>::type;
  return getMap<PointT, Field, D, OutType>(cloud);
}

template <typename PointT>
void definePointCloud(py::object &m, const char *suffix) {
  using Cloud = pcl::PointCloud<PointT>;
  using Type =
      py::class_<Cloud, std::shared_ptr<Cloud>>; // boost::shared_ptr<Cloud>
  std::string name = fmt::format("Cloud_{}", suffix);
  auto pc = Type(m, name.c_str());
  pc.def("__len__", [](Cloud &cloud) { return cloud.size(); });
  pc.def("__repr__", [suffix](Cloud &) {
    return fmt::format("ReUseX Cloud <{}>", suffix);
  });
  pc.def("__str__", [suffix](Cloud &cloud) {
    return fmt::format("Cloud {} {}", suffix, cloud.size());
  });

  pc.def_static(
      "load",
      [](std::filesystem::path path) { return ReUseX::load<PointT>(path); },
      "Load a point cloud from disk", "path"_a,
      py::call_guard<py::scoped_ostream_redirect,
                     py::scoped_estream_redirect>());
  pc.def(
      "save",
      [](typename Cloud::Ptr cloud, std::filesystem::path path) {
        ReUseX::save<PointT>(path, cloud);
      },
      "Save a point cloud to disk", "output_file"_a,
      py::return_value_policy::reference_internal,
      py::call_guard<py::scoped_ostream_redirect,
                     py::scoped_estream_redirect>());

  pc.def("toSpeckle", [](Cloud &cloud) {
    // Import library and namespaces
    auto specklepy = py::module_::import("specklepy");
    auto sp_obj = specklepy.attr("objects");
    auto sp_geo = sp_obj.attr("geometry");

    // Get types
    auto Plane = sp_geo.attr("Plane");
    auto Point = sp_geo.attr("Point");
    auto Vector = sp_geo.attr("Vector");
    auto Interval = sp_geo.attr("Interval");
    auto BBox = sp_geo.attr("Box");
    auto M_unit = sp_obj.attr("units").attr("Units").attr("m");
    auto PointCloud = sp_geo.attr("Pointcloud");

    // Create default World-XY Plane
    auto WORLDXY =
        Plane("origin"_a = Point("x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0),
              "normal"_a = Vector("x"_a = 0.0, "y"_a = 0.0, "z"_a = 1.0),
              "xdir"_a = Vector("x"_a = 1.0, "y"_a = 0.0, "z"_a = 0.0),
              "ydir"_a = Vector("x"_a = 0.0, "y"_a = 1.0, "z"_a = 0.0));

    // Compute AABB
    PointT min;
    PointT max;
    pcl::getMinMax3D(cloud, min, max);

    auto bbox =
        BBox("basePlane"_a = WORLDXY,
             "xSize"_a = Interval("start"_a = min.x, "end"_a = max.x),
             "ySize"_a = Interval("start"_a = min.y, "end"_a = max.y),
             "zSize"_a = Interval("start"_a = min.z, "end"_a = max.z),
             "volume"_a = (max.x - min.x) * (max.y - min.y) * (max.z - min.z));

    // Create views for point values and colors
    std::vector<size_t> sizes(cloud.size(), 3);
    auto points = getMap<PointT, pcl::fields::x, 3>(cloud);
    auto colors = getMap<PointT, pcl::fields::rgba, 1>(cloud);

    // Initialize Speckle PointCloud
    // TODO: Make this more efficient.
    // This currently cast the Eigen::Matrix -> Numpy Array -> Python List
    auto sp_cloud = PointCloud(
        "points"_a =
            py::cast(points.template reshaped<Eigen::RowMajor>().transpose())
                .attr("tolist")(),
        "colors"_a = py::cast(colors).attr("tolist")(),
        "sizes"_a = py::cast(sizes), "bbox"_a = bbox, "units"_a = M_unit);

    // Set chunkable attribures
    sp_cloud.attr("add_chunkable_attrs")("points"_a = 31250, "colors"_a = 62500,
                                         "sizes"_a = 62500);

    sp_cloud.attr("add_detachable_attrs")(
        py::list(py::make_tuple("points", "colors", "sizes")));

    // Return the final object
    return sp_cloud;
  });
}

void bind_pointcloud(py::module_ &m) {

  PYBIND11_NUMPY_DTYPE(PointT, x, y, z, normal_x, normal_y, normal_z, rgba,
                       curvature, label);
  PYBIND11_NUMPY_DTYPE(pcl::PointXYZRGBA, x, y, z, rgba);

  // TODO: Implement Base point cloud class_

  // py::class_<>(m, "Cloud", py::buffer_protocol());

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

  definePointCloud<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  definePointCloud<PointXYZRGBANormal>(m, "PointXYZRGBANormal");
  definePointCloud<PointT>(m, "PointTXYZRGBALNormal");
}
