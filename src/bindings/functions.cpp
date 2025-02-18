#include "functions/all.hh"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

// #include <pybind11/chrono.h>
// #include <pybind11/complex.h>
// #include <pybind11/eigen.h>
// #include <pybind11/functional.h>
// #include <pybind11/numpy.h>
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
// #include <pybind11/stl/filesystem.h>
// #include <pybind11/stl_bind.h>

// #include "algorithms/all.hh"
// #include "functions/all.hh"
// #include "types/all.hh"

// #include <algorithm>
// #include <eigen3/Eigen/Core>
// #include <string>
// #include <vector>

// #include <fmt/core.h>
// #include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_functions(py::module_ &m) {
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

  // m.def("icp", &ReUseX::pair_align<pcl::PointXYZRGBA>,
  //       "Compute the the allignment between two point clouds"
  //       "source"_a,
  //       "target"_a, "filters"_a);

  // m.def("create_cloud" & ReUseX::CreateCloud,
  //       "Create a point cloud from a dataset data item."
  //       "data item"_a,
  //       "intrinsic_matrix"_a = ReUseX::ipad_intrinsic());
}
