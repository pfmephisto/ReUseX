#pragma once

#include <memory>
#include <optional>
#include <string>

#include <spdlog/spdlog.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ReUseX {

class Visualizer;
// using VisualizerPtr = std::unique_ptr<Visualizer>;

class Visualizer {
public:
  static Visualizer *getInstance();
  static bool isInitialised();
  ~Visualizer() {
    // spdlog::trace("Calling destructor on Viewer");
    if (instance)
      instance.reset();
  };
  static void resetViewer();
  void wait() const;
  void step() const;

  template <typename T>
  void showCloud(typename pcl::PointCloud<T>::ConstPtr,
                 std::optional<std::string> = {});

  // Depending on the implemented Viewers this allows for retrieving the
  // underlying object.
  template <typename T> T *getViewer() const;

  // Delete copy constructor and assignment operator
  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

private:
  Visualizer() { spdlog::trace("Calling constructor on Viewer"); };

  static void initialise();
  static std::unique_ptr<Visualizer> instance;
};

} // namespace ReUseX
