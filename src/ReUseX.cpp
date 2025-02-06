#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "algorithms/all.hh"
#include "functions/all.hh"
#include "types/all.hh"

#include <eigen3/Eigen/Core>
#include <sstream>
#include <string>
#include <tuple>

#include <fmt/format.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_core, m) {

  // Header
  m.doc() = R"pbdoc(
        ReUseX - Reuse Explorer
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";

  // Attributes
  //    m.attr("default_params") = ReUseX::plane_fitting_parameters();

  PYBIND11_NUMPY_DTYPE(PointT, x, y, z, normal_x, normal_y, normal_z, rgba,
                       curvature, label);

  /// Classes

  // Dastatset
  /// @brief A handle for accessing raw data, like the strayscanner export.
  py::class_<ReUseX::Dataset>(m, "Dataset")
      .def(py::init<const std::string &>())
      .def(py::init(
	[](std::string & path, std::vector<ReUseX::Field> & fields){
	  return ReUseX::Dataset(std::filesystem::path(path), {ReUseX::Field::ODOMETRY});
	}))
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
      .def("__len__", &ReUseX::Dataset::size)
      .def_property_readonly("size", &ReUseX::Dataset::size)
      .def_property_readonly("color_size", &ReUseX::Dataset::color_size)
      .def_property_readonly("depth_size", &ReUseX::Dataset::depth_size)
      .def_property_readonly("name", &ReUseX::Dataset::name)
      .def("display", &ReUseX::Dataset::display, "Display the dataset",
           "name"_a, "show"_a = true)
      .def("__repr__", [](ReUseX::Dataset const& d) -> std::string {
	  return fmt::format("Dataset: {}", d.name());
	})
      ;

  /// @brief Data is the indevidual frames that the dataset provieds.
  /// Think of the data-set as a clollection of data packages.
  py::class_<ReUseX::Data>(m, "Data")
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
      .def_property_readonly(
          "pose",
          [](const ReUseX::Data &d) { return d.get<ReUseX::Field::POSES>().matrix(); })
      .def("__repr__", [](ReUseX::Data const &d) { return fmt::format("Data Item"); });

  /// @brief The collection of data types a data package can provide.
  /// They are passed to the Dataset on construction to limmit the amount of
  /// data that will be loaded.
  py::enum_<ReUseX::Field>(m, "Field")
      .value("COLOR", ReUseX::Field::COLOR)
      // .value("Color", ReUseX::Field::COLOR)
      .value("DEPTH", ReUseX::Field::DEPTH)
      // .value("Depth", ReUseX::Field::DEPTH)
      .value("CONFIDENCE", ReUseX::Field::CONFIDENCE)
      // .value("Confidence", ReUseX::Field::CONFIDENCE)
      .value("ODOMETRY", ReUseX::Field::ODOMETRY)
      // .value("Odometry", ReUseX::Field::ODOMETRY)
      .value("IMU", ReUseX::Field::IMU)
      // .value("Imu", ReUseX::Field::IMU)
      .value("POSES", ReUseX::Field::POSES)
      // .value("Poses", ReUseX::Field::POSES)
      .export_values();

  /// @brief CV Mat, used to hold image and Depth data
  py::class_<cv::Mat>(m, "Mat", py::buffer_protocol())
      .def_buffer([](cv::Mat &m) -> py::buffer_info {
        auto buffer = py::buffer_info();
        buffer.ptr = m.data;                     /* Pointer to buffer */
        buffer.itemsize = sizeof(unsigned char); /* Size of one scalar */
        buffer.format = pybind11::format_descriptor<unsigned char>::
            format();    /* Python struct-style format descriptor */
        buffer.ndim = 3; /* Number of dimensions */
        buffer.shape = {m.rows, m.cols, m.channels()}; /* Buffer dimensions */
        buffer.strides = {
            (long)sizeof(unsigned char) * m.channels() * m.cols,
            (long)sizeof(unsigned char) * m.channels(),
            (long)sizeof(
                unsigned char)}; /* Strides (in bytes) for each index */
        buffer.readonly = true;  /* Buffer is read-write */
        return buffer;
      })
      .def("__repr__", [](cv::Mat const& _ ) -> std::string { return fmt::format("Image buffer");})
      .def("toImage", [](cv::Mat & mat, bool rotate ) {
	py::module_ PIL = py::module_::import("PIL");
	py::module_ Image = PIL.attr("Image");
	if (rotate)
	  cv::rotate(mat, mat, cv::ROTATE_90_CLOCKWISE);
	auto img = Image.attr("frombuffer")("RGB", std::make_tuple(mat.cols, mat.rows), py::bytes(reinterpret_cast<const char*>(mat.data), mat.total() * mat.elemSize()));
	return img;
       },
       "rotate"_a = true
       )
  ;

  /// @brief Matrix classe used for the other rawd data that the dataset
  /// provides.
  typedef Eigen::MatrixXd Matrix;
  typedef Matrix::Scalar Scalar;
  constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
  py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
      .def_buffer([](Matrix &m) -> py::buffer_info {
        auto buffer = py::buffer_info();
        buffer.ptr = m.data();            /* Pointer to buffer */
        buffer.itemsize = sizeof(Scalar); /* Size of one scalar */
        buffer.format =
            py::format_descriptor<Scalar>::format(); /* Python struct-style
                                                        format descriptor */
        buffer.ndim = 2;                             /* Number of dimensions */
        buffer.shape = {m.rows(), m.cols()};         /* Buffer dimensions */
        buffer.strides = {
            sizeof(Scalar) * (rowMajor ? m.cols() : 1),
            (long)sizeof(Scalar) *
                (rowMajor ? 1
                          : m.rows())}; /* Strides (in bytes) for each index */
        buffer.readonly = true;         /* Buffer is read-write */
        return buffer;
      });

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
      // .def("annotate", &ReUseX::PointCloudsInMemory::annotate,
      //     "Annotate the point clouds",
      //     "yolo_path"_a,
      //     "dataset"_a = py::none(),
      //     py::return_value_policy::reference_internal,
      //     py::call_guard<py::scoped_ostream_redirect,
      //     py::scoped_estream_redirect>()
      // )
      .def("display", &ReUseX::PointCloudsInMemory::display,
           "Display the point clouds", "show_clouds"_a = false,
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
      .def("display", &ReUseX::PointCloudsOnDisk::display,
           "Display the point clouds", "show_clouds"_a = false,
           py::return_value_policy::reference_internal,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
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

  /// @brief Plane class
  py::class_<ReUseX::Plane>(m, "ReUseX")
      .def(py::init<>())
      .def(py::init<const float, const float, const float, const float>())
      .def(py::init<const float, const float, const float, const float,
                    const float, const float, const float>())
      .def("__repr__",
           [](const ReUseX::Plane &p) {
             std::stringstream ss;
             ss << "Plane A(" << p.a() << ") B(" << p.b() << ") C(" << p.c()
                << ") D(" << p.d() << ")";
             return ss.str();
           })
      .def_readwrite("origin", &ReUseX::Plane::origin)
      .def("normal", &ReUseX::Plane::normal);

  /// @brief Position
  py::class_<ReUseX::Point>(m, "Pos")
      .def(py::init<const float, const float, const float>())
      .def(("__repr__"),
           [](const ReUseX::Point &p) {
             std::stringstream ss;
             ;
             ss << "Pos X=" << p.x() << " y=" << p.y() << " z=" << p.z();
             return ss.str();
           })
      .def("x", &ReUseX::Point::x)
      .def("y", &ReUseX::Point::y)
      .def("z", &ReUseX::Point::z);

  /// @brief Position
  py::class_<ReUseX::Point2>(m, "Pos2D")
      .def(py::init<const float, const float>())
      .def(("__repr__"),
           [](const ReUseX::Point2 &p) {
             std::stringstream ss;
             ;
             ss << "Pos2D X=" << p.x() << " y=" << p.y();
             return ss.str();
           })
      .def("x", &ReUseX::Point2::x)
      .def("y", &ReUseX::Point2::y);

  /// @brief Vector
  py::class_<ReUseX::Vector>(m, "Vec")
      .def(py::init<const float, const float, const float>())
      .def(("__repr__"), [](const ReUseX::Vector &v) {
        std::stringstream ss;
        ;
        ss << "Vec X=" << v.x() << " y=" << v.y() << " z=" << v.z();
        return ss.str();
      });

  /// @brief Direction, Normalised vector
  py::class_<ReUseX::Direction>(m, "Dir")
      .def(py::init<const float, const float, const float>())
      .def(("__repr__"),
           [](const ReUseX::Direction &v) {
             std::stringstream ss;
             ;
             ss << "Dir X=" << v.dx() << " y=" << v.dy() << " z=" << v.dz();
             return ss.str();
           })
      // .def_property_readonly("valid", [](const ReUseX::Direction &v){ return
      // tg::normalize_safe((ReUseX::Vector)v) !=  ReUseX::Vector::zero;
      //  })
      ;

  py::class_<ReUseX::AABB>(m, "AABB")
      .def(py::init<const ReUseX::Point, const ReUseX::Point>())
      .def("__repr__",
           [](const ReUseX::AABB &a) {
             std::stringstream ss;
             ss << "AABB min(" << a.min().x() << ", " << a.min().y() << ", "
                << a.min().z() << ") max(" << a.max().x() << ", " << a.max().y()
                << ", " << a.max().z() << ")";
             return ss.str();
           })
      .def("volume",
           [](const ReUseX::AABB &a) {
             return (a.max().x() - a.min().x()) * (a.max().y() - a.min().y()) *
                    (a.max().z() - a.min().z());
           })
      .def("center",
           [](const ReUseX::AABB &a) {
             return ReUseX::Point((a.max().x() + a.min().x()) / 2,
                                  (a.max().y() + a.min().y()) / 2,
                                  (a.max().z() + a.min().z()) / 2);
           })
      .def("xInterval",
           [](const ReUseX::AABB &a) {
             return std::make_tuple(a.min().x(), a.max().x());
           })
      .def("yInterval",
           [](const ReUseX::AABB &a) {
             return std::make_tuple(a.min().y(), a.max().y());
           })
      .def("zInterval", [](const ReUseX::AABB &a) {
        return std::make_tuple(a.min().z(), a.max().z());
      });

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

  m.def(
      "display_brep",
      [](ReUseX::Brep &brep, std::string name = "Brep", bool show = true) {
        brep.display(name, show);
      },
      "Display a Brep");

  // TODO: Those should be moved to their respecive classes
  // Functions
  m.def("parse_dataset", &ReUseX::parse_Dataset,
        "Parse a StrayScanner scan in to a point cloud"
        "dataset"_a,
        "output_path"_a, "start"_a = 0, "stop"_a = nullptr, "step"_a = 5);

  m.def("slam", &ReUseX::slam, "SLAM", "dataset"_a,
        py::call_guard<py::scoped_ostream_redirect,
                       py::scoped_estream_redirect>());

  m.def(
      "compute_normals",
      [](std::filesystem::path path) {
        ReUseX::compute_normals<pcl::PointXYZRGBA, PointT>(path);
      },
      "Compute the normals of a point cloud"
      "path"_a,
      py::call_guard<py::scoped_ostream_redirect,
                     py::scoped_estream_redirect>());

  m.def("extract_instances", &ReUseX::extract_instances,
        "Extract the instance for a point cloud"
        "point_cloud"_a);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

} // PYBIND11_MODULE(ReUseX, m)

// namespace PYBIND11_NAMESPACE { namespace detail {
//     template <> struct type_caster<ReUseX::Brep> {
//     public:
//         /**
//          * This macro establishes the name 'Brep' in
//          * function signatures and declares a local variable
//          * 'value' of type ReUseX::Brep
//          */
//         PYBIND11_TYPE_CASTER(ReUseX::Brep, const_name("Brep"));

//         /**
//          * Conversion part 1 (Python->C++): convert a PyObject into a
//          ReUseX::Brep
//          * instance or return false upon failure. The second argument
//          * indicates whether implicit conversions should be applied.
//          */
//         bool load(handle src, bool) {
//             // /* Extract PyObject from handle */
//             // PyObject *source = src.ptr();
//             // /* Try converting into a Python integer value */
//             // PyObject *tmp = PyNumber_Long(source);
//             // if (!tmp)
//             //     return false;
//             // /* Now try to convert into a C++ int */
//             // value.long_value = PyLong_AsLong(tmp);
//             // Py_DECREF(tmp);
//             // /* Ensure return code was OK (to avoid out-of-range errors
//             etc) */
//             // return !(value.long_value == -1 && !PyErr_Occurred());

//             py::print("Loading Brep from Python");

//             return false;
//         }

//         /**
//          * Conversion part 2 (C++ -> Python): convert an ReUseX::Brep
//          instance into
//          * a Python object. The second and third arguments are used to
//          * indicate the return value policy and parent object (for
//          * ``return_value_policy::reference_internal``) and are generally
//          * ignored by implicit casters.
//          */
//         static handle cast(ReUseX::Brep src, return_value_policy /* policy
//         */, handle /* parent */) {

//             py::print("Casting Brep to Python");
//             py::object rhinolib = py::module_::import("rhino3dm");
//             py::object rhino_brep = rhinolib.attr("Brep")();

//             return rhino_brep;
//             // return PyLong_FromLong(src.long_value);
//         }
//     };
// }} // namespace PYBIND11_NAMESPACE::detail
