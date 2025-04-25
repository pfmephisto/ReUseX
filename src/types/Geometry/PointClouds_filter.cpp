#include "PointCloud.hh"
#include "PointClouds.hh"
#include "core/spdmon.hh"

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <vector>

namespace ReUseX {
template <typename T>
PointClouds<T> PointClouds<T>::filter(std::uint32_t value) {

  // Only keep highest confidence
  ///////////////////////////////////////////////////////////////////////////////
  auto monitor = spdmon::LoggerProgress("Filtering", data.size());
  spdlog::stopwatch sw;
#pragma omp parallel for
  for (size_t i = 0; i < data.size(); i++) {
    if constexpr (std::is_same<T, std::string>::value) {
      auto cloud = PointCloud::load(data[i]);
      cloud.filter(value);
      cloud.save(data[i]);
    } else if constexpr (std::is_same<T, PointCloud>::value) {
      data[i].filter(value);
    }
    ++monitor;
  }
  spdlog::info("Filtered {} clouds in {} seconds", data.size(), sw);

  return *this;
}

template PointCloudsInMemory PointCloudsInMemory::filter(std::uint32_t value);
template PointCloudsOnDisk PointCloudsOnDisk::filter(std::uint32_t value);

} // namespace ReUseX
