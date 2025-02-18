#include "types/all.hh"
#include <pybind11/pybind11.h>
// #include <pybind11/chrono.h>
// #include <pybind11/complex.h>
// #include <pybind11/functional.h>
// #include <pybind11/iostream.h>
// #include <pybind11/numpy.h>
#include <pybind11/stl.h>
// #include <pybind11/stl/filesystem.h>
// #include <pybind11/stl_bind.h>
// #include <pybind11/eigen.h>

#include <eigen3/Eigen/Core>

// #include <algorithm>
// #include <string>
// #include <vector>

// #include <fmt/core.h>
#include <fmt/format.h>

#include <opencv4/opencv2/core.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_dataset(py::module_ &);
void bind_dataitem(py::module_ &);
void bind_pointcloud(py::module_ &);
void bind_pointclouds(py::module_ &);
void bind_brep(py::module_ &);

void bind_types(py::module_ &m) {

  // Attributes
  //    m.attr("default_params") = ReUseX::plane_fitting_parameters();

  // PYBIND11_NUMPY_DTYPE(Odometry, timestamp, frame, x, y, z, qx, qy, qz, qw);
  // PYBIND11_NUMPY_DTYPE(IMU, timestamp, a_x, a_y, a_z, alpha_x, alpha_y,
  // alpha_z);

  bind_dataset(m);    // This is the StrayScanner Dataset
  bind_dataitem(m);   // This is one data package from the StrayScanner Dataset
  bind_pointcloud(m); // This is the Point Cloud data type
  bind_pointclouds(m);
  bind_brep(m);

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
      .def("__repr__",
           [](cv::Mat const &_) -> std::string {
             return fmt::format("Image buffer");
           })
      .def(
          "toImage",
          [](cv::Mat &mat, bool rotate) {
            py::module_ PIL = py::module_::import("PIL");
            py::module_ Image = PIL.attr("Image");
            cv::Mat mat_copy = mat.clone();
            if (rotate)
              cv::rotate(mat_copy, mat_copy, cv::ROTATE_90_CLOCKWISE);
            auto img = Image.attr("frombuffer")(
                "RGB", std::make_tuple(mat_copy.cols, mat_copy.rows),
                py::bytes(reinterpret_cast<const char *>(mat_copy.data),
                          mat_copy.total() * mat_copy.elemSize()));
            return img;
          },
          "rotate"_a = true);

  ///// @brief Matrix classe used for the other rawd data that the dataset
  ///// provides.
  // typedef Eigen::MatrixXd Matrix;
  // typedef Matrix::Scalar Scalar;
  // constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
  // py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
  //     .def_buffer([](Matrix &m) -> py::buffer_info {
  //       auto buffer = py::buffer_info();
  //       buffer.ptr = m.data();            /* Pointer to buffer */
  //       buffer.itemsize = sizeof(Scalar); /* Size of one scalar */
  //       buffer.format =
  //           py::format_descriptor<Scalar>::format(); /* Python struct-style
  //                                                       format descriptor */
  //       buffer.ndim = 2;                             /* Number of dimensions
  //       */ buffer.shape = {m.rows(), m.cols()};         /* Buffer dimensions
  //       */ buffer.strides = {
  //           sizeof(Scalar) * (rowMajor ? m.cols() : 1),
  //           (long)sizeof(Scalar) *
  //               (rowMajor ? 1
  //                         : m.rows())}; /* Strides (in bytes) for each index
  //                         */
  //       buffer.readonly = true;         /* Buffer is read-write */
  //       return buffer;
  //     });

  ///// @brief Plane class
  // py::class_<ReUseX::Plane>(m, "ReUseX")
  //     .def(py::init<>())
  //     .def(py::init<const float, const float, const float, const float>())
  //     .def(py::init<const float, const float, const float, const float,
  //                   const float, const float, const float>())
  //     .def("__repr__",
  //          [](const ReUseX::Plane &p) {
  //            std::stringstream ss;
  //            ss << "Plane A(" << p.a() << ") B(" << p.b() << ") C(" << p.c()
  //               << ") D(" << p.d() << ")";
  //            return ss.str();
  //          })
  //     .def_readwrite("origin", &ReUseX::Plane::origin)
  //     .def("normal", &ReUseX::Plane::normal);

  ///// @brief Position
  // py::class_<ReUseX::Point>(m, "Pos")
  //     .def(py::init<const float, const float, const float>())
  //     .def(("__repr__"),
  //          [](const ReUseX::Point &p) {
  //            std::stringstream ss;
  //            ;
  //            ss << "Pos X=" << p.x() << " y=" << p.y() << " z=" << p.z();
  //            return ss.str();
  //          })
  //     .def("x", &ReUseX::Point::x)
  //     .def("y", &ReUseX::Point::y)
  //     .def("z", &ReUseX::Point::z);

  ///// @brief Position
  // py::class_<ReUseX::Point2>(m, "Pos2D")
  //     .def(py::init<const float, const float>())
  //     .def(("__repr__"),
  //          [](const ReUseX::Point2 &p) {
  //            std::stringstream ss;
  //            ;
  //            ss << "Pos2D X=" << p.x() << " y=" << p.y();
  //            return ss.str();
  //          })
  //     .def("x", &ReUseX::Point2::x)
  //     .def("y", &ReUseX::Point2::y);

  ///// @brief Vector
  // py::class_<ReUseX::Vector>(m, "Vec")
  //     .def(py::init<const float, const float, const float>())
  //     .def(("__repr__"), [](const ReUseX::Vector &v) {
  //       std::stringstream ss;
  //       ;
  //       ss << "Vec X=" << v.x() << " y=" << v.y() << " z=" << v.z();
  //       return ss.str();
  //     });

  ///// @brief Direction, Normalised vector
  // py::class_<ReUseX::Direction>(m, "Dir")
  //     .def(py::init<const float, const float, const float>())
  //     .def(("__repr__"),
  //          [](const ReUseX::Direction &v) {
  //            std::stringstream ss;
  //            ;
  //            ss << "Dir X=" << v.dx() << " y=" << v.dy() << " z=" << v.dz();
  //            return ss.str();
  //          })
  //     // .def_property_readonly("valid", [](const ReUseX::Direction &v){
  //     return
  //     // tg::normalize_safe((ReUseX::Vector)v) !=  ReUseX::Vector::zero;
  //     //  })
  //     ;

  // py::class_<ReUseX::AABB>(m, "AABB")
  //     .def(py::init<const ReUseX::Point, const ReUseX::Point>())
  //     .def("__repr__",
  //          [](const ReUseX::AABB &a) {
  //            std::stringstream ss;
  //            ss << "AABB min(" << a.min().x() << ", " << a.min().y() << ", "
  //               << a.min().z() << ") max(" << a.max().x() << ", " <<
  //               a.max().y()
  //               << ", " << a.max().z() << ")";
  //            return ss.str();
  //          })
  //     .def("volume",
  //          [](const ReUseX::AABB &a) {
  //            return (a.max().x() - a.min().x()) * (a.max().y() - a.min().y())
  //            *
  //                   (a.max().z() - a.min().z());
  //          })
  //     .def("center",
  //          [](const ReUseX::AABB &a) {
  //            return ReUseX::Point((a.max().x() + a.min().x()) / 2,
  //                                 (a.max().y() + a.min().y()) / 2,
  //                                 (a.max().z() + a.min().z()) / 2);
  //          })
  //     .def("xInterval",
  //          [](const ReUseX::AABB &a) {
  //            return std::make_tuple(a.min().x(), a.max().x());
  //          })
  //     .def("yInterval",
  //          [](const ReUseX::AABB &a) {
  //            return std::make_tuple(a.min().y(), a.max().y());
  //          })
  //     .def("zInterval", [](const ReUseX::AABB &a) {
  //       return std::make_tuple(a.min().z(), a.max().z());
  //     });
}
