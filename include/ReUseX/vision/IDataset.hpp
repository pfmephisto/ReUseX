// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <ReUseX/vision/IData.hpp>
#include <filesystem>
#include <opencv4/opencv2/core/mat.hpp>
#include <span>
#include <sqlite3.h>
#include <vector>

namespace ReUseX::vision {
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

  /* Constructs a new IDataset object. The constructor takes a path to the
   * SQLite database file, which is used to initialize the dataset. The
   * constructor opens the database connection and retrieves the list of sample
   * IDs from the database, which are stored in the ids_ member variable. The
   * destructor closes the database connection. The size method returns the
   * number of samples in the dataset, which is equal to the size of the ids_
   * vector. The get method retrieves a sample by its index, which is used to
   * look up the corresponding sample ID in the ids_ vector. The get method then
   * retrieves the image and label for the sample from the database, and returns
   * a Pair containing a unique pointer to an IData object that represents the
   * sample, and the index of the sample in the dataset. The save method takes a
   * span of Pairs, which allows the caller to save a batch of samples to the
   * database. The save method iterates over the span of Pairs, and for each
   * Pair, it retrieves the IData object and its index, and saves the image and
   * label for the sample to the database. The save method returns true if all
   * samples were saved successfully, and false otherwise. The getImage and
   * saveImage methods are used internally by the get and save methods to
   * retrieve and save images to the database. The getImage method retrieves the
   * image data for a sample from the database, and returns it as a cv::Mat
   * object. The saveImage method saves the image data for a sample to the
   * database, and returns true if the image was saved successfully, and false
   * otherwise.
   * @param dbPath The path to the SQLite database file.
   */
  IDataset(std::filesystem::path dbPath);

  /* Virtual destructor to ensure proper cleanup of derived classes. */
  ~IDataset();

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
  cv::Mat getImage(const std::size_t index) const;

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
  bool saveImage(const std::size_t index, const cv::Mat &image);

    protected:
  /* The database connection for the dataset. The db_ member variable is a
   * pointer to a sqlite3 object that represents the connection to the SQLite
   * database. The db_ member variable is initialized in the constructor, where
   * the database connection is opened using the sqlite3_open function. The db_
   * member variable is used by the get and save methods to execute SQL queries
   * to retrieve and save data samples to the database. The db_ member variable
   * is closed in the destructor, where the sqlite3_close function is called to
   * close the database connection.
   */
  sqlite3 *db_ = nullptr;

  /* The list of sample IDs in the dataset. The ids_ member variable is a vector
   * of integers that contains the IDs of the samples in the dataset. The ids_
   * member variable is initialized in the constructor, where a SQL query is
   * executed to retrieve the list of sample IDs from the database, and the
   * results are stored in the ids_ vector. The ids_ member variable is used by
   * the get and save methods to look up the corresponding sample ID for a given
   * index when retrieving and saving samples to the database.
   */
  std::vector<int> ids_ = {};
};
} // namespace ReUseX::vision
