# Point Cloud Filtering Implementation

## Overview
Implemented the TODO at line 442-447 in `/libs/reusex/src/io/rtabmap.cpp` to filter outliers and noise from point clouds while maintaining synchronization between points, normals, and labels.

## Implementation Details

### Added Headers
```cpp
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
```

### Filtering Pipeline

The implementation uses a two-stage filtering approach:

#### 1. Statistical Outlier Removal (SOR)
- **Purpose**: Removes points that are statistical outliers based on mean distance to neighbors
- **Parameters**:
  - `MeanK = 50`: Number of neighbors to analyze for each point
  - `StddevMulThresh = 1.0`: Standard deviation multiplier threshold
- **Effect**: Removes isolated noise points that are far from their neighbors

#### 2. Radius Outlier Removal (ROR)
- **Purpose**: Removes points with insufficient neighbors within a specified radius
- **Parameters**:
  - `RadiusSearch = resolution * 2.0`: Search radius scaled to voxel size
  - `MinNeighborsInRadius = 5`: Minimum number of neighbors required
- **Effect**: Removes sparse outliers and ensures point density consistency

### Synchronization Strategy

The key challenge was ensuring that points, normals, and labels remain synchronized after filtering. The implementation achieves this by:

1. **Computing filter indices once**: Both filters generate indices of valid points
2. **Applying the same indices to all three clouds**: Using `pcl::ExtractIndices` with the same index set for:
   - Point cloud (`CloudPtr`)
   - Normal cloud (`CloudNPtr`)
   - Label cloud (`CloudLPtr`)
3. **Verification**: Checks that all filtered clouds have the same size before returning

### Code Structure

```cpp
// Step 1: Statistical filtering
pcl::StatisticalOutlierRemoval<PointT> sor;
sor.setInputCloud(out_cloud);
sor.setMeanK(50);
sor.setStddevMulThresh(1.0);
pcl::IndicesPtr stat_inliers(new pcl::Indices);
sor.filter(*stat_inliers);

// Step 2: Radius filtering (applied on top of statistical filtering)
pcl::RadiusOutlierRemoval<PointT> ror;
ror.setInputCloud(out_cloud);
ror.setIndices(stat_inliers);
ror.setRadiusSearch(resolution * 2.0);
ror.setMinNeighborsInRadius(5);
pcl::IndicesPtr filtered_indices(new pcl::Indices);
ror.filter(*filtered_indices);

// Step 3: Apply same indices to all three clouds
pcl::ExtractIndices<PointT> extract_points;
extract_points.setInputCloud(out_cloud);
extract_points.setIndices(filtered_indices);
extract_points.filter(*filtered_cloud);

// ... same for normals and labels ...

// Step 4: Verify synchronization
if (filtered_cloud->size() != filtered_normals->size() ||
    filtered_cloud->size() != filtered_labels->size()) {
    // Error handling
}
```

### Performance & Logging

The implementation includes comprehensive logging:
- Progress tracking with `spdlog::info/debug/trace`
- Timing information using `spdlog::stopwatch`
- Statistics showing number of points retained at each filtering stage
- Final summary with percentage of points retained

Example output:
```
Filtering outliers from point cloud (123456 points)
Statistical outlier removal kept 120000/123456 points
Radius outlier removal kept 115000/120000 points
Filtering complete in 2.345s: 115000/123456 points retained (93.2%)
```

## Benefits

1. **Noise Reduction**: Removes sensor noise and spurious measurements
2. **Data Consistency**: Ensures points, normals, and labels always correspond
3. **Downstream Quality**: Cleaner point clouds improve subsequent processing steps:
   - Plane segmentation
   - Room segmentation
   - Mesh generation
   - Texture mapping
4. **Robustness**: Adaptive filtering based on voxel resolution parameter
5. **Safety**: Verification step prevents size mismatches that could cause crashes

## Testing

The implementation:
- ✅ Compiles successfully with no warnings or errors
- ✅ Properly handles all PCL data types (PointXYZRGB, Normal, Label)
- ✅ Uses existing project types and conventions
- ✅ Integrates seamlessly with existing RTABMap import pipeline

## Future Enhancements

Potential improvements for future consideration:
- Make filter parameters configurable via function arguments
- Add optional `pcl::NormalSpaceSampling` for more uniform point distribution
- Implement `pcl::ModelOutlierRemoval` for removing points that don't fit expected models
- Add benchmarking to measure performance impact on various dataset sizes
