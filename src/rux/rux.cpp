#include "rux/annotate.hpp"
#include "rux/cellcomplex.hpp"
#include "rux/export.hpp"
#include "rux/seg-planes.hpp"
#include "rux/seg-rooms.hpp"
#include "rux/surface_reconstruction.hpp"

#include "ReUseX/about.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

int main(int argc, char **argv) {
  spdlog::set_level(spdlog::level::warn); // Default level

  CLI::App app{
      "rux: ReUseX a tool for processing interior lidar scans of buildings."};
  app.get_formatter()->column_width(40);

  app.add_flag(
         "-v, --verbose",
         [](int count) {
           spdlog::set_level(static_cast<spdlog::level::level_enum>(3 - count));
           spdlog::info("Verbosity level: {}",
                        spdlog::level::to_string_view(spdlog::get_level()));
         },
         "Increase verbosity, use -vv & -vvv for more details.")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));
  app.add_flag(
      "-V, --version",
      [](bool /*count*/) {
        std::cout << "rux version: " << ReUseX::VERSION << std::endl;
        exit(0);
      },
      "Show version information");
  app.add_flag(
      "	-L, --license",
      [](bool /*count*/) {
        std::cout << ReUseX::LICENSE_TEXT << std::endl;
        exit(0);
      },
      "Show license information");

  setup_subcommand_annotate(app);
  setup_subcommand_export(app);
  setup_subcommand_seg_planes(app);
  setup_subcommand_seg_rooms(app);
  setup_subcommand_cellcomplex(app);
  setup_subcommand_surface_reconstruction(app);

  app.add_subcommand("assemble", "Assemble multiple scans.")->callback([]() {
    spdlog::warn("The assemble command is not yet implemented.");
    exit(1);
  });

  app.require_subcommand(/* min */ 1, /* max */ 1);

  argv = app.ensure_utf8(argv);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }
  return 0;
}
