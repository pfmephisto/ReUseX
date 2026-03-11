# RTABMapDatabase Design

This document details the design and implementation of the `RTABMapDatabase` class, which provides unified access to RTABMap databases with custom segmentation label storage.

## Purpose

RTABMapDatabase serves as the primary interface for:
1. **Image retrieval** from RTABMap SLAM databases
2. **Segmentation label storage** via custom SQL table
3. **SLAM graph access** for pose and map information

## Architecture

### Pimpl Pattern

Uses the Pimpl (Pointer to Implementation) idiom to hide RTABMap dependencies:

```cpp
// Public header (RTABMapDatabase.hpp)
class RTABMapDatabase {
public:
    RTABMapDatabase(const std::string& dbPath);
    cv::Mat getImage(int nodeId);
    cv::Mat getLabels(int nodeId);
    // ...
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// Implementation file (RTABMapDatabase.cpp)
class RTABMapDatabase::Impl {
    rtabmap::Rtabmap rtabmap_;
    std::vector<int> nodeIds_;
    // RTABMap-specific details...
};
```

**Benefits**:
- Public headers don't include RTABMap (faster compilation)
- ABI stability
- Clear separation of interface and implementation

## Database Schema

### RTABMap Tables (Native)

RTABMap provides these tables:
- `Node`: SLAM graph nodes (id, pose, timestamp)
- `Data`: Image data, depth, calibration
- `Link`: Constraints between nodes
- Others: Map, Feature, Statistics, etc.

### Custom Segmentation Table

Added by ReUseX:

```sql
CREATE TABLE IF NOT EXISTS Segmentation (
    node_id INTEGER PRIMARY KEY,
    labels BLOB NOT NULL,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (node_id) REFERENCES Node(id) ON DELETE CASCADE
);
```

**Fields**:
- `node_id`: References RTABMap Node.id (one-to-one)
- `labels`: PNG-encoded CV_16U image with segmentation labels
- `created_at`: Timestamp for versioning/debugging

## Image Handling

### Rotation Convention

RTABMap stores images in device orientation. ReUseX applies automatic rotation:

```cpp
cv::Mat getImage(int nodeId) {
    auto rawImage = rtabmap_->getImage(nodeId);
    cv::Mat rotated;
    cv::rotate(rawImage, rotated, cv::ROTATE_90_CLOCKWISE);
    return rotated;
}
```

**Rationale**: Device captures in landscape; users expect portrait orientation

**Consistency**: Rotation applied to both images AND labels

### Format

- **Color space**: RGB (OpenCV default: BGR, converted as needed)
- **Data type**: CV_8UC3 for images
- **Resolution**: As captured (typically 1920x1080 or similar)

## Label Encoding

### Storage Format (Database)

Labels stored as PNG-encoded CV_16U:
- **Type**: CV_16U (16-bit unsigned integer)
- **Encoding**: PNG compression via cv::imencode
- **Offset**: Values stored with +1 offset
  - 0 → Background/unlabeled
  - 1 → Class 0
  - 2 → Class 1
  - etc.

### API Format (User-Facing)

Labels returned as CV_32S:
- **Type**: CV_32S (32-bit signed integer)
- **Encoding**: Raw values
- **Convention**:
  - -1 → Background/unlabeled
  - 0 → Class 0
  - 1 → Class 1
  - etc.

### Encoding/Decoding

```cpp
// Saving: API (CV_32S) → Storage (CV_16U + 1)
void saveLabels(int nodeId, const cv::Mat& labels) {
    // Rotate 90° counterclockwise (inverse of getImage)
    cv::Mat rotated;
    cv::rotate(labels, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    
    // Convert CV_32S → CV_16U with +1 offset
    cv::Mat storage;
    rotated.convertTo(storage, CV_16U, 1.0, 1.0);  // +1 offset
    
    // Encode as PNG
    std::vector<uchar> buffer;
    cv::imencode(".png", storage, buffer);
    
    // Store in database...
}

// Loading: Storage (CV_16U) → API (CV_32S - 1)
cv::Mat getLabels(int nodeId) {
    // Load from database
    auto buffer = loadFromDB(nodeId);
    
    // Decode PNG
    cv::Mat storage = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
    
    // Convert CV_16U → CV_32S with -1 offset
    cv::Mat labels;
    storage.convertTo(labels, CV_32S, 1.0, -1.0);  // -1 offset
    
    // Rotate 90° clockwise (same as images)
    cv::Mat rotated;
    cv::rotate(labels, rotated, cv::ROTATE_90_CLOCKWISE);
    
    return rotated;
}
```

**Why the offset?**
- PNG format doesn't handle negative values
- 0 in CV_16U is unambiguous "no label"
- API uses -1 for clarity (common ML convention)

## Key Methods

### Constructor
```cpp
RTABMapDatabase(const std::string& dbPath);
```
- Opens existing database or creates new one
- Initializes RTABMap instance
- Creates Segmentation table if missing
- Throws on failure (file not found, corrupt DB)

### getImage
```cpp
cv::Mat getImage(int nodeId);
```
- Retrieves image for given node
- Applies 90° clockwise rotation
- Returns CV_8UC3 RGB image
- Returns empty Mat if node not found

### getLabels
```cpp
cv::Mat getLabels(int nodeId);
```
- Retrieves segmentation labels
- Decodes from PNG (CV_16U → CV_32S)
- Applies -1 offset and 90° rotation
- Returns empty Mat if no labels for node

### saveLabels
```cpp
void saveLabels(int nodeId, const cv::Mat& labels);
```
- Stores segmentation labels for node
- Applies 90° counterclockwise rotation
- Converts CV_32S → CV_16U with +1 offset
- Encodes as PNG for compression
- Upserts into Segmentation table

### getNodeIds
```cpp
std::vector<int> getNodeIds();
```
- Returns all node IDs in database
- Cached for performance (populated on first call)
- Ordered by node ID

### getGraph
```cpp
std::multimap<int, rtabmap::Link> getGraph();
```
- Provides access to SLAM graph
- Returns node links (edges)
- Used for pose estimation and mapping

## Performance Considerations

### Caching
- **Node IDs**: Cached in memory (populated once)
- **Images**: Not cached (loaded on demand)
- **Labels**: Not cached (loaded on demand)

**Rationale**: Images/labels are large; caching would consume memory. Database (SQLite) provides efficient access.

### Compression
Labels stored as PNG:
- Reduces database size (~10x for typical labels)
- Fast decode (OpenCV imdecode)
- Lossless (critical for segmentation)

### Database Connection
Single rtabmap::Rtabmap instance per RTABMapDatabase:
- Thread-safe (SQLite with proper flags)
- Connection pooling not needed (short-lived queries)

## Thread Safety

- **RTABMap API**: Thread-safe for read operations
- **SQLite**: Configured for multi-threaded access
- **Recommendation**: One RTABMapDatabase instance per thread for writes

## Error Handling

### Exceptions (Unrecoverable)
- Database file not found
- Corrupt database
- RTABMap initialization failure

### Return Values (Expected)
- Empty Mat for missing images/labels
- Empty vector for databases with no nodes

### Logging
- spdlog integration for diagnostics
- Warnings for missing data
- Debug info for database operations

## Usage Examples

### Basic Usage
```cpp
#include <ReUseX/io/RTABMapDatabase.hpp>

auto db = ReUseX::io::RTABMapDatabase("scan.db");

// Get all nodes
auto nodeIds = db.getNodeIds();

// Process each image
for (int nodeId : nodeIds) {
    auto image = db.getImage(nodeId);
    // ... run segmentation ...
    cv::Mat labels = runSegmentation(image);
    
    // Save results
    db.saveLabels(nodeId, labels);
}
```

### With Dataset
```cpp
auto db = std::make_shared<RTABMapDatabase>("scan.db");
auto dataset = ReUseX::vision::IDataset(db);

// Use with TensorRT
auto trtDataset = ReUseX::vision::TensorRTDataset(dataset);
```

## Future Enhancements

### Potential Improvements
1. **Caching strategy**: LRU cache for recently accessed images
2. **Batch operations**: Load multiple images in single query
3. **Metadata storage**: Store model version, confidence scores
4. **Label versioning**: Multiple segmentation versions per image
5. **Compression options**: Allow JPEG for images (lossy but smaller)

### API Extensions
- `getImageMetadata()`: Camera calibration, timestamp
- `getDepth()`: Depth image access
- `getPose()`: Node pose from SLAM graph
- `getBatch()`: Efficient multi-image loading

## Related Files

- `libs/reusex/include/ReUseX/io/RTABMapDatabase.hpp`: Public interface
- `libs/reusex/src/ReUseX/io/RTABMapDatabase.cpp`: Implementation
- `libs/reusex/src/ReUseX/io/rtabmap.cpp`: RTABMap integration (legacy)

## Questions?

For questions about RTABMapDatabase:
1. Review this document and code comments
2. Check unit tests (future): `tests/unit/io/test_rtabmap_database.cpp`
3. Open an issue with the `database` label
