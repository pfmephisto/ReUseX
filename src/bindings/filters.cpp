#include "types/Filters.hh"
#include "types/Geometry/PointCloud.hh"

#include <pcl/filters/voxel_grid.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/point_struct_traits.h>
#include <pcl/type_traits.h>

#include <fmt/format.h>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename PointT> auto defineFilter(py::module_ &m, const char *name) {

  using FilterType = pcl::Filter<PointT>;

  auto F =
      py::class_<FilterType, std::shared_ptr<FilterType>, pcl::PCLBase<PointT>>(
          m, fmt::format("Filter_{}", name).c_str());

  F.def("__repr__",
        [name](FilterType &) { return fmt::format("Filter<{}>", name); });
  F.def("__str__",
        [name](FilterType &) { return fmt::format("Filter<{}>", name); });
  F.def("filter", static_cast<void (FilterType::*)(pcl::PointCloud<PointT> &)>(
                      &FilterType::filter));
}

template <typename PointT>
auto defineFilterIndices(py::module_ &m, const char *name) {

  using FilterType = pcl::FilterIndices<PointT>;
  auto F =
      py::class_<FilterType, std::shared_ptr<FilterType>, pcl::Filter<PointT>>(
          m, fmt::format("FilterIndices_{}", name).c_str());

  return F;
}

template <typename PointT>
auto defineVoxelGridFilter(py::module_ &m, const char *name) {

  using FilterType = pcl::VoxelGrid<PointT>;
  auto F =
      py::class_<FilterType, std::shared_ptr<FilterType>, pcl::Filter<PointT>>(
          m, fmt::format("VolxelGrid_{}", name).c_str());
  F.def(py::init());
  F.def("set_leaf_size",
        py::overload_cast<float, float, float>(
            &pcl::VoxelGrid<PointT>::setLeafSize),
        "x"_a, "y"_a, "z"_a);
  return F;
}

template <typename PointT>
auto defineExtractIndicesFilter(py::module_ &m, const char *name) {
  using FilterType = pcl::ExtractIndices<PointT>;

  auto F = py::class_<FilterType, std::shared_ptr<FilterType>,
                      pcl::FilterIndices<PointT>>(
      m, fmt::format("ExtractIndices_{}", name).c_str());
  F.def(py::init<bool>(), "extract_removed_indices"_a = false);
  F.def("__repr__",
        [name](FilterType &) { return fmt::format("Filter<{}>", name); });
  F.def("__str__",
        [name](FilterType &) { return fmt::format("Filter<{}>", name); });

  return F;
}

template <typename PointT>
auto defienConditionalRemoval(py::module_ &m, const char *name) {
  using FilterType = pcl::ConditionalRemoval<PointT>;

  auto F =
      py::class_<FilterType, std::shared_ptr<FilterType>, pcl::Filter<PointT>>(
          m, fmt::format("ConditionalRemoval_{}", name).c_str());

  F.def(py::init());
  F.def("__repr__", [name](FilterType &) {
    return fmt::format("ConditionalRemoval<{}>", name);
  });
  F.def("__str__", [name](FilterType &) {
    return fmt::format("ConditionalRemoval<{}>", name);
  });

  return F;
}

void bind_filters(py::module_ &m) {

  defineFilter<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  defineFilter<PointXYZRGBANormal>(m, "PointXYZRGBANormal");

  defineFilterIndices<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  defineFilterIndices<PointXYZRGBANormal>(m, "PointXYZRGBANormal");

  defineExtractIndicesFilter<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  defineExtractIndicesFilter<PointXYZRGBANormal>(m, "PointXYZRGBANormal");

  defineVoxelGridFilter<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  defineVoxelGridFilter<PointXYZRGBANormal>(m, "PointXYZRGBANormal");

  defienConditionalRemoval<pcl::PointXYZRGBA>(m, "PointXYZRGBA");
  defienConditionalRemoval<PointXYZRGBANormal>(m, "PointXYZRGBANormal");

  m.def("HighConfidenceFilter_PointXYZRGBA",
        &ReUseX::Filters::HighConfidenceFilter<pcl::PointXYZRGBA>);
  m.def("HighConfidenceFilter_PointXYZRGBA",
        &ReUseX::Filters::HighConfidenceFilter<PointXYZRGBANormal>);
  m.def("HighConfidenceFilter_PointXYZRGBANormal",
        &ReUseX::Filters::HighConfidenceFilter<PointXYZRGBANormal>);
}
