#include "core/all.hh"

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl/filesystem.h>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include <pybind11/stl.h>

#define PYBIND11_DETAILED_ERROR_MESSAGES

#ifdef _OPENMP
/// Multi-threading - yay!
#include <omp.h>
#else
/// Macros used to disguise the fact that we do not have multithreading enabled.
#define omp_get_thread_num() 0
#define omp_get_num_threads() 1
#endif

namespace py = pybind11;
using namespace pybind11::literals;
using namespace std::literals::chrono_literals;

void bind_functions(py::module_ &m) {
  // TODO: Those should be moved to their respecive classes
  // Functions
  m.def("parse_dataset", &ReUseX::parse_Dataset,
        "Parse a StrayScanner scan in to a point cloud"
        "dataset"_a,
        "output_path"_a, "start"_a = 0, "stop"_a = nullptr, "step"_a = 5);

  m.def(
      "compute_normals",
      [](std::filesystem::path path) {
        return ReUseX::compute_normals<pcl::PointXYZRGBA, PointXYZRGBANormal>(
            path);
      },
      // static_cast<void (*)(std::filesystem::path)>(
      //     &ReUseX::compute_normals<pcl::PointXYZRGBA, PointT>),
      // py::overload_cast<void (*)(std::filesystem::path)>(
      //     &ReUseX::compute_normals<pcl::PointXYZRGBA, PointT>),
      "Compute the normals of a point cloud"
      "path"_a,
      py::call_guard<py::scoped_ostream_redirect,
                     py::scoped_estream_redirect>());

  m.def(
      "compute_normals",
      [](typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
        return ReUseX::compute_normals<pcl::PointXYZRGBA, PointXYZRGBANormal>(
            cloud);
      },
      // static_cast<pcl::PointCloud<PointT>::Ptr (*)(
      //     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)>(
      //     &ReUseX::compute_normals<pcl::PointXYZRGBA, PointT>),
      //  py::overload_cast<typename pcl::PointCloud<PointT>::Ptr (*)(
      //      typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)>(
      //      &ReUseX::compute_normals<pcl::PointXYZRGBA, PointT>),
      "Compute the normals of a point cloud", "cloud"_a,
      py::call_guard<py::scoped_ostream_redirect,
                     py::scoped_estream_redirect>());

  m.def("extract_instances", &ReUseX::extract_instances,
        "Extract the instance for a point cloud"
        "point_cloud"_a);

  m.def("icp", &ReUseX::icp<pcl::PointXYZRGBA>,
        "Compute the the allignment between two point clouds"
        "source"_a,
        "target"_a, "filters"_a = py::list(), "max_corespondance"_a = 0.2);

  m.def("icp", &ReUseX::icp<PointXYZRGBANormal>,
        "Compute the the allignment between two point clouds"
        "source"_a,
        "target"_a, "filters"_a = py::list(), "max_corespondance"_a = 0.2);

  m.def("create_cloud", &ReUseX::CreateCloud,
        "Create a point cloud from a dataset data item.", "data_item"_a,
        "intrinsic_matrix"_a = ReUseX::ipad_intrinsic(),
        "seq_index"_a = py::none());

  m.def("get_pos",
        [](py::array_t<double, py::array::c_style | py::array::forcecast>
               odometry) {
          auto buf = odometry.request();

          std::optional<int> none = {};
          py::slice s1(none, none, 1);
          py::slice s2(2, 5, 1);

          auto tup = buf.ndim == 3 ? py::make_tuple(s1, s1, s2)
                                   : py::make_tuple(s1, s2);

          return odometry[tup];
        });
  m.def("get_quat",
        [](py::array_t<double, py::array::c_style | py::array::forcecast>
               odometry) {
          auto buf = odometry.request();

          std::optional<int> none = {};
          py::slice s1(none, none, 1);
          py::slice s2(5, none, 1);

          auto tup = buf.ndim == 3 ? py::make_tuple(s1, s1, s2)
                                   : py::make_tuple(s1, s2);

          return odometry[tup];
        });

  m.def("write_graph", &ReUseX::write_graph,
        "Parse Dataset in to0 graph"
        "path"_a,
        "dataset"_a, "indices"_a, "group_size"_a = 3,
        "solver_name"_a = "lm_fix6_3_csparse", "kernel_name"_a = "Huber",
        "max_iterations"_a = 20, "maxCorrespondence"_a = 1,
        "delta_value"_a = 2.5,
        "information_matrix"_a = Eigen::Matrix<double, 6, 6>::Identity(),
        py::call_guard<py::scoped_ostream_redirect,
                       py::scoped_estream_redirect>());

  m.def("register_logger", &spdlog::register_logger);
  m.def("set_default_logger", &spdlog::set_default_logger);

  m.def("set_level", [](int level) {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
  });

  auto atexit = py::module_::import("atexit");
  atexit.attr("register")(py::cpp_function([]() {
    // perform cleanup here -- this function is called with the GIL held
    // Flush all the loggers before shutdown
    spdlog::shutdown();
  }));
}
