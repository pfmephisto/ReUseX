#define PCL_NO_PRECOMPILE

#include "types/Geometry/PointCloud.hh"
#include "types/Yolo.hh"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ReUseX {

std::unordered_map<std::string, std::vector<PointCloud>>
extract_instances(PointCloud const &cloud) {

  auto instances = std::unordered_map<std::string, std::vector<PointCloud>>();

  // Get all semantic labels
  auto semantic_labels_set = std::unordered_set<int>();
  for (auto point : cloud->points)
    semantic_labels_set.insert(point.label);

  // (cloud->points.begin(), cloud->points.end(), [](const PointT& point){
  //         return point.semantic;
  //     });

  // Remove unknown label
  if (std::find(semantic_labels_set.begin(), semantic_labels_set.end(), 0) !=
      semantic_labels_set.end())
    semantic_labels_set.erase(-1);

  auto semantic_labels =
      std::vector<int>(semantic_labels_set.begin(), semantic_labels_set.end());

  for (size_t i = 0; i < semantic_labels.size(); i++) {

    auto label = semantic_labels[i];

    // Get all instance labels
    auto instance_labels_set = std::unordered_set<int>();
    for (auto point : cloud->points) {
      if (point.label == label)
        instance_labels_set.insert(point.label);
    }

    auto instance_labels = std::vector<int>(instance_labels_set.begin(),
                                            instance_labels_set.end());

    // Remove unknown label
    if (std::find(instance_labels_set.begin(), instance_labels_set.end(), 0) !=
        instance_labels_set.end())
      instance_labels_set.erase(0);

    // Extract Semantic name and initialize cloud
    std::string label_str = Yolov8Seg::GetClassName(label);
    instances[label_str] = std::vector<PointCloud>(instance_labels.size());

    for (size_t j = 0; j < instance_labels.size(); j++) {
      auto instance = instance_labels[j];

      instances[label_str][j] = PointCloud();

      std::copy_if(
          cloud->points.begin(), cloud->points.end(),
          std::back_inserter(instances[label_str][j]->points),
          [label, instance](const PointCloud::Cloud::PointType &point) {
            return point.label == label;
          });
    }
  }

  return instances;
}
} // namespace ReUseX
