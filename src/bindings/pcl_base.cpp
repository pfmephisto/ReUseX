#include "types/Geometry/PointCloud.hh"

#include <iterator>
#include <memory>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>

#include <pcl/pcl_base.h>
#include <pcl/type_traits.h>
#include <pcl/types.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

#define PYBIND11_DETAILED_ERROR_MESSAGES

template <typename PointT>
auto definePCLBase(py::module_ &m, const char *name) {

  using FilterType = pcl::PCLBase<PointT>;

  using IndicesConstPtr = std::shared_ptr<const pcl::Indices>;

  auto BC = py::class_<FilterType, std::shared_ptr<FilterType>>(
      m, fmt::format("PCLBase_{}", name).c_str());
  // BC.def("set_indices",
  //        static_cast<void (pcl::PCLBase<PointT>::*)(const IndicesConstPtr
  //        &)>(
  //            &pcl::PCLBase<PointT>::setIndices),
  //        "Set indecies");
  BC.def(
      "set_indices",
      [](FilterType &self,
         py::array_t<int, py::array::c_style | py::array::forcecast> array) {
        std::vector<int> vi = py::cast<std::vector<int>>(array);

        std::shared_ptr<pcl::Indices> indices =
            std::shared_ptr<pcl::Indices>(new pcl::Indices());

        std::copy(vi.begin(), vi.end(), std::back_inserter(*indices));

        return self.setIndices(indices);
      },
      "Set indecies");

  return BC;
}

void bind_pclbase(py::module_ &m) {

  // Define base classes
  definePCLBase<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  definePCLBase<PointXYZRGBANormal>(m, "PointXYZRGBANormal");
}
