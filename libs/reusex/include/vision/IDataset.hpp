// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "reusex/vision/IData.hpp"

#include <opencv2/core/mat.hpp>

#include <filesystem>
#include <memory>
#include <span>
#include <vector>

// Forward declaration
namespace reusex {
class ProjectDB;
}

namespace reusex::vision {
/* Interface for datasets. A dataset is a collection of data samples, where each
 * sample consists of an image and a label. The dataset is stored in a SQLite
 * database, where each sample is stored as a row in a table. The table has the
 * following columns: - id: an integer primary key that uniquely identifies the
 * sample - image: a blob that contains the image data - label: an integer that
 * represents the label of the sample. The dataset provides methods for
 * retrieving samples and saving new samples to the database. The get method
 * retrieves a sample by its index, and the save method saves a batch of samples
 * to the database. The dataset also provides methods for retrieving and saving
 * images, which are used internally by the get and save methods. The dataset is
 * designed to be used with the IData interface, which represents a single data
 * sample. The IData interface provides methods for accessing the image and
 * label of a sample, and for saving the sample to the database. The dataset is
 * intended to be used in machine learning applications, where it can be used to
 * train and evaluate models on a collection of labeled images. */
class IDataset {
    public:
  /* A pair of a data sample and its index. The data sample is represented as a
   * unique pointer to an IData object, and the index is a size_t that
   * represents the position of the sample in the dataset. The get method
   * returns a Pair, which allows the caller to access both the data sample
   * and its index. The save method takes a span of Pairs, which allows the
   * caller to save a batch of samples to the database. */
  using Pair = std::pair<std::unique_ptr<IData>, size_t>;

  /* Constructs a new IDataset object with a shared database instance.
   *
   * This constructor allows multiple IDataset instances to share the same
   * database connection. The database is managed by shared_ptr, so it will
   * remain open as long as any IDataset instance references it.
   *
   * @param database Shared pointer to ProjectDB instance
   */
  explicit IDataset(std::shared_ptr<ProjectDB> database);

  /* Constructs a new IDataset object by opening a database at the given path.
   *
   * This convenience constructor creates a new ProjectDB instance internally
   * and stores it as a shared_ptr. The database connection is managed by the
   * IDataset and will be closed when the last reference is destroyed.
   *
   * @param dbPath The path to the ReUseX project database file.
   */
  explicit IDataset(std::filesystem::path dbPath);

  /* Virtual destructor to ensure proper cleanup of derived classes. */
  virtual ~IDataset() = default;

  /* Returns the number of samples in the dataset. The size method returns the
   * number of samples in the dataset, which is equal to the size of the ids_
   * vector. The size method is used by the caller to determine how many samples
   * are available in the dataset, and to iterate over the samples using their
   * indices. The size method is a const method, which means that it does not
   * modify the state of the IDataset object.
   * @return The number of samples in the dataset.
   */
  size_t size() const;

  /* Retrieves a sample by its index. The get method takes an index as input,
   * which is used to look up the corresponding sample ID in the ids_ vector.
   * The get method then retrieves the image and label for the sample from the
   * database, and returns a Pair containing a unique pointer to an IData object
   * that represents the sample, and the index of the sample in the dataset. The
   * get method is a const method, which means that it does not modify the state
   * of the IDataset object. The get method is a pure virtual method, which
   * means that it must be implemented by derived classes.
   * @param index The index of the sample to retrieve.
   * @return A Pair containing a unique pointer to an IData object that
   * represents the sample, and the index of the sample in the dataset.
   */
  virtual Pair get(const std::size_t index) const = 0;

  /* Saves a batch of samples to the database. The save method takes a span of
   * Pairs as input, which allows the caller to save a batch of samples to the
   * database. The save method iterates over the span of Pairs, and for each
   * Pair, it retrieves the IData object and its index, and saves the image and
   * label for the sample to the database. The save method returns true if all
   * samples were saved successfully, and false otherwise. The save method is a
   * pure virtual method, which means that it must be implemented by derived
   * classes.
   * @param data A span of Pairs, where each Pair contains a unique pointer to
   * an IData object that represents a sample, and the index of the sample in
   * the dataset.
   * @return true if all samples were saved successfully, and false otherwise.
   */
  virtual bool save(const std::span<Pair> &data) = 0;

    protected:
  /* Retrieves the image data for a sample from the database. The getImage
   * method takes an index as input, which is used to look up the corresponding
   * sample ID in the ids_ vector. The getImage method then retrieves the image
   * data for the sample from the database, and returns it as a cv::Mat object.
   * The getImage method is a const method, which means that it does not modify
   * the state of the IDataset object. The getImage method is used internally by
   * the get method to retrieve the image data for a sample when constructing an
   * IData object to represent the sample.
   * @param index The index of the sample whose image data to retrieve.
   * @return A cv::Mat object containing the image data for the sample.
   */
  cv::Mat image(const std::size_t index) const;

  /* Saves the image data for a sample to the database. The saveImage method
   * takes an index and a cv::Mat object as input, which represent the index of
   * the sample and the image data to save, respectively. The saveImage method
   * saves the image data for the sample to the database, and returns true if
   * the image was saved successfully, and false otherwise. The saveImage method
   * is used internally by the save method to save the image data for a sample
   * when saving a batch of samples to the database.
   * @param index The index of the sample whose image data to save.
   * @param image A cv::Mat object containing the image data to save for the
   * sample.
   * @return true if the image was saved successfully, and false otherwise.
   */
  bool save_image(const std::size_t index, const cv::Mat &image);

  /* Access to the underlying database for subclasses.
   *
   * Subclasses can use this to access database functionality beyond the
   * basic getImage/saveImage interface if needed.
   *
   * @return Shared pointer to the ProjectDB instance
   */
  std::shared_ptr<ProjectDB> database() const;

    private:
  /* Shared pointer to the project database. Multiple IDataset instances can
   * share the same database connection. The database connection is managed
   * via RAII and will be closed when the last reference is destroyed.
   */
  std::shared_ptr<ProjectDB> db_;

  /* Cached list of node IDs in the dataset. This is populated once during
   * construction by querying the database. The IDs are used to map from
   * dataset indices (0, 1, 2, ...) to RTABMap node IDs.
   */
  std::vector<int> ids_;
};
} // namespace reusex::vision
