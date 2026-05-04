// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/semantic_images.hpp"
#include "global-params.hpp"
#include "spdmon.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <spdlog/spdlog.h>

#include <opencv2/imgcodecs.hpp>
#include <pcl/common/colors.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace reusex;

void setup_subcommand_export_semantic_images(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportSemanticImagesOptions>();
  auto *sub = parent.add_subcommand(
      "semantic-images",
      "Export semantic segmentation images as colored PNGs");

  sub->footer(R"(
DESCRIPTION:
  Exports all semantic segmentation images stored in the project database as
  color-coded PNG files. Each label index is mapped to a distinct color using
  the Glasbey lookup table, which provides maximally distinct colors for
  categorical data. Background pixels (label -1) are rendered as black.

OUTPUT:
  Files are written as: <output_dir>/<prefix>_<node_id>.png
  One PNG per sensor frame that has a stored segmentation image.

EXAMPLES:
  rux export semantic-images -o ./labels          # Export to ./labels/
  rux export semantic-images -o ./out --prefix seg # Use custom prefix

NOTES:
  - Requires segmentation images to have been stored via 'rux annotate'
  - Colors are reproducible: same label index always gets the same color
  - Glasbey LUT provides up to 256 visually distinct colors
)");

  sub->add_option("-o, --output", opt->output_dir,
                  "Directory to write PNG files")
      ->default_val(opt->output_dir);

  sub->add_option("--prefix", opt->prefix,
                  "Filename prefix for output files (<prefix>_<id>.png)")
      ->default_val(opt->prefix);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export semantic-images subcommand");
    return run_subcommand_export_semantic_images(*opt, *global_opt);
  });
}

int run_subcommand_export_semantic_images(SubcommandExportSemanticImagesOptions const &opt,
                                          const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Exporting semantic images from project: {}", project_path.string());

  try {
    ProjectDB db(project_path, true);

    auto ids = db.segmentation_image_ids();
    if (ids.empty()) {
      spdlog::warn("No segmentation images found in project database");
      return RuxError::SUCCESS;
    }

    fs::create_directories(opt.output_dir);

    int max_id = *std::max_element(ids.begin(), ids.end());
    int pad_width = static_cast<int>(std::to_string(max_id).size());

    spdmon::LoggerProgress progress("Exporting", static_cast<unsigned>(ids.size()));

    int exported = 0;
    for (int id : ids) {
      cv::Mat labels = db.segmentation_image(id); // CV_32S, -1=background

      cv::Mat colored(labels.size(), CV_8UC3, cv::Scalar(0, 0, 0));

      labels.forEach<int32_t>([&](int32_t label, const int pos[]) {
        if (label < 0)
          return; // background stays black
        auto c = pcl::GlasbeyLUT::at(static_cast<unsigned>(label) % pcl::GlasbeyLUT::size());
        colored.at<cv::Vec3b>(pos[0], pos[1]) = cv::Vec3b(c.b, c.g, c.r); // OpenCV is BGR
      });

      std::ostringstream id_str;
      id_str << std::setw(pad_width) << std::setfill('0') << id;
      fs::path out_path = opt.output_dir / (opt.prefix + "_" + id_str.str() + ".png");
      if (!cv::imwrite(out_path.string(), colored)) {
        spdlog::error("Failed to write {}", out_path.string());
        return RuxError::IO;
      }

      spdlog::debug("Wrote {}", out_path.string());
      ++progress;
      ++exported;
    }

    spdlog::info("Exported {} semantic image(s) to {}", exported, opt.output_dir.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Semantic image export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
