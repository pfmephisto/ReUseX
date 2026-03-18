#pragma once
#include <opencv2/opencv.hpp>

#include <map>
#include <string>
#include <vector>

// FreeType headers
#include <ft2build.h>
#include FT_FREETYPE_H

namespace ReUseX::vision::osd {

/// @brief FreeType-based text renderer for OpenCV images.
///
/// CvxText renders UTF-8 encoded strings onto cv::Mat images using a TrueType
/// font loaded via the FreeType library.  Multiple font faces can be managed
/// through a single instance — the first call to putText or getTextSize for a
/// given font path will load and cache the face automatically.
class CvxText {
    public:
  /// @brief Load the primary font face from the given file path.
  /// @param font_path Path to a TrueType (.ttf) or OpenType font file.
  /// @throws std::runtime_error if the FreeType library or face cannot be
  /// initialized.
  explicit CvxText(const char *font_path);

  virtual ~CvxText();

  /// @brief Render a UTF-8 string onto an OpenCV image.
  /// @param img      Destination image (modified in-place).
  /// @param text     UTF-8 string to render.
  /// @param org      Top-left drawing origin in image coordinates.
  /// @param color    BGR text color.
  /// @param font_size Glyph height in pixels.
  void putText(cv::Mat &img, const std::string &text, cv::Point org,
               cv::Scalar color, int font_size);

  /// @brief Compute the approximate bounding-box dimensions of a rendered
  /// string.
  /// @param text      UTF-8 string to measure.
  /// @param font_size Glyph height in pixels.
  /// @param[out] w        Approximate rendered width in pixels.
  /// @param[out] h        Approximate rendered height (ascent) in pixels.
  /// @param[out] baseline Distance from the baseline to the bottom of the
  /// bounding box.
  void getTextSize(const std::string &text, int font_size, int *w, int *h,
                   int *baseline);

    private:
  CvxText(const CvxText &) = delete;
  CvxText &operator=(const CvxText &) = delete;

  /// @brief Load (or retrieve from cache) a FreeType face for the given path.
  FT_Face getFace(const char *font_path);

  /// @brief Decode a UTF-8 string into a vector of UCS-4 code points.
  void utf8_to_ucs4(const std::string &str, std::vector<long> &ucs4);

    private:
  FT_Library m_library;
  std::map<std::string, FT_Face> m_faces;
};

} // namespace ReUseX::vision::osd
