#include "visualizer.hh"
#include "types/Filters.hh"

#include <creators.h>
#include <memory>

#ifdef _OPENMP
/// Multi-threading - yay!
#include <omp.h>
#else
/// Macros used to disguise the fact that we do not have multithreading enabled.
#define omp_get_thread_num() 0
#define omp_get_num_threads() 1
#endif

#include <optional>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/impl/point_types.hpp>
#include <spdlog/spdlog.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

using namespace std::chrono_literals;

namespace ReUseX {

using PCLVisualizer = pcl::visualization::PCLVisualizer;

std::unique_ptr<ReUseX::Visualizer> ReUseX::Visualizer::instance =
    std::unique_ptr<ReUseX::Visualizer>();
std::unique_ptr<PCLVisualizer> pcl_viewer;

ReUseX::Visualizer *Visualizer::getInstance() {
  spdlog::trace("Get Viewer instance");

  // static ReUseX::Visualizer instance;
  spdlog::debug("Instace: {}", fmt::ptr(instance));

  if (!instance) {
    instance.reset(new ReUseX::Visualizer());
    Visualizer::initialise();
  }

  return instance.get();
}

bool skip_view = false;
std::function<void(const pcl::visualization::KeyboardEvent &)>
    keyboardSkipCallback(
        [&skip_view](const pcl::visualization::KeyboardEvent &event) {
          if (event.keyUp()) {
            switch (event.getKeyCode()) {
            case 'm':
              skip_view = true;
              break;
            }
          }
        });

void Visualizer::initialise() {
  spdlog::trace("Initialising Viewer");

  //// Initialize the pcl Viewer
  if (!pcl_viewer) {
    pcl_viewer = std::unique_ptr<PCLVisualizer>(new PCLVisualizer("ReUseX"));
    pcl_viewer->setBackgroundColor(255, 255, 255);

    pcl_viewer->addCoordinateSystem(1.0);
    pcl_viewer->initCameraParameters();
    pcl_viewer->registerKeyboardCallback(keyboardSkipCallback);

    spdlog::trace("PCL Visualizer Initialized");
  }
}

void Visualizer::resetViewer() {
  // spdlog::trace("Resetting Visualizer");
  if (pcl_viewer) {
    pcl_viewer.reset();
    spdlog::trace("PCL Visualizer reset");
  }

  instance.reset();
}

template <>
void Visualizer::showCloud<pcl::PointXYZRGBA>(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
    std::optional<std::string> name_) {

  using PointT = pcl::PointXYZRGBA;

  if (!pcl_viewer)
    return;

  std::string name = name_.has_value() ? name_.value() : cloud->header.frame_id;

  pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
  auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
  filter->setInputCloud(cloud);
  filter->filter(*cloud_);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(
      cloud_);

  if (!rgb.isCapable())
    spdlog::warn("Not able to shwo color: {}", name);

  pcl_viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_, rgb, name);
  pcl_viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
}

template <> PCLVisualizer *Visualizer::getViewer<PCLVisualizer>() const {
  spdlog::trace("Getting PCL Visualizer");
  return pcl_viewer.get();
}

void Visualizer::wait() const {
  spdlog::trace("Wait for interupt");
  while (!pcl_viewer->wasStopped() && !skip_view) {
    step();
    std::this_thread::sleep_for(100ms);
  }
  skip_view = false;
}
bool Visualizer::skip() const { return skip_view; }

void Visualizer::step() const { pcl_viewer->spinOnce(100); }

bool Visualizer::isInitialised() {
  spdlog::trace("Checking if viewer is initialised");

  if (omp_get_num_threads() > 1)
    return false;

  spdlog::debug("Instace: {}", fmt::ptr(instance));
  return bool(instance);
};

} // namespace ReUseX
