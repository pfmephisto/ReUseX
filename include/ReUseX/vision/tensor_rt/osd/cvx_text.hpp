#pragma once
#include <map>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

// FreeType Headers
#include <ft2build.h>
#include FT_FREETYPE_H

namespace ReUseX::vision::tensor_rt {
class CvxText {
    public:
  explicit CvxText(const char *font_path);
  virtual ~CvxText();

  void putText(cv::Mat &img, const std::string &text, cv::Point org,
               cv::Scalar color, int font_size);

  // --- NEW: Added method to get text size ---
  /**
  * @brief Calculates the approximate bounding box size of the rendered text
  *
  @param text The UTF-8 string to be calculated
  * @param font_size The font size (in pixels)
  * @param[out] w Stores the calculated width
  * @param[out] h Stores the calculated height
  * @param[out] baseline Stores the calculated baseline
  */
  void getTextSize(const std::string &text, int font_size, int *w, int *h,
                   int *baseline);

    private:
  CvxText(const CvxText &) = delete;
  CvxText &operator=(const CvxText &) = delete;

  FT_Face getFace(const char *font_path);
  void utf8_to_ucs4(const std::string &str, std::vector<long> &ucs4);

    private:
  FT_Library m_library;
  std::map<std::string, FT_Face> m_faces;
};
} // namespace ReUseX::vision::tensor_rt
