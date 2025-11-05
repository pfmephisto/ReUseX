// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/cellcomplex.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/color.h>
#include <fmt/std.h>

#include <ReUseX/geometry/CellComplex.hpp>
#include <ReUseX/geometry/Solidifier.hpp>
#include <ReUseX/geometry/regularization.hpp>
#include <ReUseX/geometry/utils.hpp>
#include <ReUseX/io/reusex.hpp>

#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <filesystem>
#include <latch>
#include <mutex>
#include <thread>

namespace fs = std::filesystem;

void setup_subcommand_cellcomplex(CLI::App &app) {

  auto opt = std::make_shared<SubcommandCellcomplexOptions>();
  auto *sub = app.add_subcommand(
      "cellcomplex",
      "This tool computes the best fit volumes form a set of input planes.");

  sub->add_option("cloud", opt->cloud_path_in, "Path to the input cloud file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->cloud_path_in);

  sub->add_option("normals", opt->normals_path_in,
                  "Path to the input normals file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->normals_path_in);

  sub->add_option("planes", opt->planes_path_in,
                  "Path to the input planes file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->planes_path_in);

  sub->add_option("rooms", opt->rooms_path_in, "Path to the input labe file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->rooms_path_in);

  sub->add_option("output", opt->output_out, "Path to the output file.")
      ->default_val(opt->output_out);

  sub->add_option("-a, --angle", opt->angle_threshold,
                  "Angle threshold for plane pairing in degrees.")
      ->default_val(opt->angle_threshold)
      ->check(CLI::Range(1.0, 180.0));

  sub->add_option("-l, --distance", opt->distance_threshold,
                  "Distance threshold for plane pairing.")
      ->default_val(opt->distance_threshold)
      ->check(CLI::Range(0.001, 3.0));

  sub->add_option("-t, --threshold", opt->search_threshold,
                  "Distance threshold for plane pairing.")
      ->default_val(opt->search_threshold)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("-o, --offset", opt->new_plane_offset,
                  "Distance threshold for new plane creation.")
      ->default_val(opt->new_plane_offset)
      ->check(CLI::Range(0.01, 1.0));

  sub->add_flag("-d, --visualize", opt->display, "Display the result.")
      ->default_val(opt->display);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_cellcomplex");
    return run_subcommand_cellcomplex(*opt);
  });
};

int run_subcommand_cellcomplex(SubcommandCellcomplexOptions const &opt) {

  // TODO: Move the visualizer to a separate thread
  // The goal is to be able to use the viewer while the computation is running

  // TODO: Make sure the input planse are cleaned
  // - optinally apply shape regularization to enforce orthogonality
  //   [link](https://doc.cgal.org/latest/Shape_regularization/index.html#Chapter_Shape_Regularization)

  // INFO: Setup viewer and vizualiztion queue
  using VisualizerPtr = std::shared_ptr<pcl::visualization::PCLVisualizer>;
  using VizTask = std::function<void(VisualizerPtr)>;

  std::queue<VizTask> task_queue;
  std::mutex queue_mutex;

  // Synchronization for viewport initialization
  std::latch barrier(1);

  int vp_1, vp_2, vp_3, vp_4;
  std::thread viz_thread([&]() {
    // Early exit if visualization is not enabled
    if (!opt.display)
      return;

    spdlog::trace("Visualization thread started");
    VisualizerPtr viewer =
        std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

    // INFO: Create viewports 1
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_1);
    viewer->addCoordinateSystem(1.0, "reference_vp1", vp_1);

    // INFO: Create viewports 2
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_2);
    viewer->addCoordinateSystem(1.0, "reference_vp2", vp_2);

    // INFO: Create viewports 3
    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_3);
    viewer->addCoordinateSystem(1.0, "reference_vp3", vp_3);

    // INFO: Create viewports 4
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_4);
    viewer->addCoordinateSystem(1.0, "reference_vp4", vp_4);

    viewer->setBackgroundColor(0, 0, 0, 1);
    viewer->initCameraParameters();

    // Notify main thread that viewports are ready
    barrier.count_down();

    // Main loop
    // Spin until the viewer is closed
    while (!viewer->wasStopped()) {
      // Execute queued functions
      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        while (!task_queue.empty()) {
          task_queue.front()(viewer); // call function
          task_queue.pop();
        }
      }
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  // Wait for visualization thread to finish initializing viewports
  barrier.wait(); // blocks until count_down called

  assert(fs::exists(opt.cloud_path_in) &&
         "Input point cloud file does not exist");
  assert(fs::exists(opt.normals_path_in) &&
         "Input normals file does not exist");
  assert(fs::exists(opt.planes_path_in) && "Input planes file does not exist");
  assert(fs::exists(opt.rooms_path_in) && "Input rooms file does not exist");

  CloudPtr cloud(new Cloud);
  spdlog::trace("Reading {:<8} file: {}", "input", opt.cloud_path_in);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying point cloud with {} points", cloud->size());
      viewer->addPointCloud<PointT>(cloud, "cloud", vp_1);
      viewer->resetCameraViewpoint("cloud");
    });
  }

  CloudNPtr normals(new CloudN);
  spdlog::trace("Reading {:<8} file: {}", "normals", opt.normals_path_in);
  pcl::io::load<NormalT>(opt.normals_path_in.string(), *normals);

  // TODO: Currently not used
  // Instead we are reading planes form the .planes file
  // CloudLPtr planes_cloud(new CloudL);
  // spdlog::trace("<eading {:<8} file: {}", "planes", opt.planes_path_in);
  // pcl::io::load<LabelT>(opt.planes_path_in.string() * planes_cloud);

  CloudLPtr rooms(new CloudL);
  spdlog::trace("Reading {:<8} file: {}", "rooms", opt.rooms_path_in);
  pcl::io::load<LabelT>(opt.rooms_path_in.string(), *rooms);

  if (rooms->size() != cloud->size() || /*planes->size() != cloud->size() ||*/
      normals->size() != cloud->size()) {
    spdlog::error("Input files have inconsistent sizes:");
    return EXIT_FAILURE;
  }

  // // INFO: Create room labels from point colors
  // std::unordered_set<uint32_t> unique_labels;
  // for (const auto &pt : cloud->points)
  //   unique_labels.insert(pt.rgba);

  // spdlog::info("Found {} unique room labels", unique_labels.size());
  // std::vector<uint32_t> labels_tmp(unique_labels.begin(),
  // unique_labels.end()); std::unordered_map<uint32_t, size_t>
  // label_to_index{};

  // for (size_t i = 0; i < labels_tmp.size(); ++i)
  //   label_to_index[labels_tmp[i]] = i;

  // spdlog::info("Labels: {}", fmt::join(labels_tmp, ", "));
  // labels->points.resize(cloud->size());
  // labels->width = cloud->width;
  // labels->height = cloud->height;

  // for (size_t i = 0; i < cloud->size(); ++i)
  //   labels->points[i].label = label_to_index[cloud->points[i].rgba];

  // pcl::io::save<LabelT>(opt.labels_path.string(), *labels);

  // TODO: Rewrite reader function to be more portable
  fs::path planes_path = opt.planes_path_in;
  planes_path.replace_extension("planes");
  spdlog::trace("Reading {:<8} file: {}", "planes", planes_path);
  std::vector<pcl::ModelCoefficients> planes_coeff = {};
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids_in;
  std::vector<IndicesPtr> inliers = {};
  ReUseX::io::read(planes_path, planes_coeff, centroids_in, inliers);

  // Convert to Eigen::Vector4d
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      planes;
  planes.reserve(planes_coeff.size());
  std::transform(planes_coeff.begin(), planes_coeff.end(),
                 std::back_inserter(planes), [](auto const &c) {
                   return Eigen::Vector4d(c.values[0], c.values[1], c.values[2],
                                          c.values[3]);
                 });
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      centroids;
  centroids.reserve(centroids_in.size());
  std::transform(
      centroids_in.begin(), centroids_in.end(), std::back_inserter(centroids),
      [](auto const &c) { return Eigen::Vector3d(c[0], c[1], c[2]); });
  spdlog::trace("Number of planes read: {}", planes.size());

  planes = ReUseX::geometry::regularizePlanes<double, PointT>(planes, cloud,
                                                              inliers, 10.0);

  std::tie(planes, inliers, centroids) =
      ReUseX::geometry::merge_planes(planes, inliers, centroids, cloud);
  spdlog::debug("Number of planes after merging: {}", planes.size());

  planes = ReUseX::geometry::force_orthogonal_planes(planes);
  spdlog::debug("Number of planes after forcing orthogonality: {}",
                planes.size());

  auto pairs = ReUseX::geometry::make_pairs(
      planes, inliers, centroids, opt.search_threshold, opt.new_plane_offset);
  spdlog::debug("Number of plane pairs: {}", pairs.size());

  auto [vertical, horizontal] = ReUseX::geometry::separate_planes(planes);
  spdlog::debug("Number indices in inliers [{}]",
                fmt::join(inliers | ranges::views::transform([](auto const &i) {
                            return i->size();
                          }),
                          ", "));
  spdlog::debug("Number of horizonal planes: {}", horizontal.size());
  spdlog::debug("Number of vertical planes: {}", vertical.size());
  // spdlog::debug("Planes [{}]", fmt::join(ranges::views::iota(0) |
  //                                            ranges::views::take(planes.size()),
  //                                        " "));
  // spdlog::debug("Pairs [{}]",
  //               fmt::join(pairs | ranges::views::transform([](auto const &p)
  //               {
  //                           return fmt::format("({}, {})", p.first,
  //                           p.second);
  //                         }),
  //                         ", "));
  spdlog::debug("Vertical planes [{}]", fmt::join(vertical, ", "));
  spdlog::debug("Horizontal planes [{}]", fmt::join(horizontal, ", "));

  // INFO: Display planes
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying planes");

      size_t count = 0;
      for (size_t i = 0; i < vertical.size(); ++i, ++count) {
        const size_t id = vertical[i];
        const auto plane = planes[id];
        const auto origin = centroids[id];
        std::string name = fmt::format("wall_{}-planes", i);
        pcl::ModelCoefficients coeff;
        coeff.values = {
            static_cast<float>(plane[0]), static_cast<float>(plane[1]),
            static_cast<float>(plane[2]), static_cast<float>(plane[3])};
        viewer->addPlane(coeff, origin[0], origin[1], origin[2], name, vp_1);
        auto color = pcl::GlasbeyLUT::at(count);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            static_cast<double>(color.r) / 255.0,
            static_cast<double>(color.g) / 255.0,
            static_cast<double>(color.b) / 255.0, name, vp_1);
        // Add text next to plane
        // const PointT p(origin[0], origin[1], origin[2]);
        // viewer->addText3D(name, p, 0.2, 1.0, 1.0, 1.0,
        //                   fmt::format("text_{}", name), vp_1);
      }
      for (size_t i = 0; i < horizontal.size(); ++i, ++count) {
        const size_t id = horizontal[i];
        const auto plane = planes[id];
        const auto origin = centroids[id];
        std::string name = fmt::format("floor_{}-planes", i);
        pcl::ModelCoefficients coeff;
        coeff.values = {
            static_cast<float>(plane[0]), static_cast<float>(plane[1]),
            static_cast<float>(plane[2]), static_cast<float>(plane[3])};
        viewer->addPlane(coeff, origin[0], origin[1], origin[2], name, vp_1);
        auto color = pcl::GlasbeyLUT::at(count);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            static_cast<double>(color.r) / 255.0,
            static_cast<double>(color.g) / 255.0,
            static_cast<double>(color.b) / 255.0, name);
        // const PointT p(static_cast<float>(origin[0]),
        //                static_cast<float>(origin[1]),
        //                static_cast<float>(origin[2]));
        // viewer->addText3D(name, p, 0.2, 1.0, 1.0, 1.0,
        //                   fmt::format("text_{}", name), vp_1);
      }
      for (const auto &[i, j] : pairs) {
        auto origin_i = centroids[i];
        auto origin_j = centroids[j];

        PointT p1, p2;
        std::string name = fmt::format("pair_{}-{}", i, j);
        p1.getVector3fMap() = origin_i.head<3>().cast<float>();
        p2.getVector3fMap() = origin_j.head<3>().cast<float>();
        viewer->addLine<PointT>(p1, p2, 0.0, 0.0, 1.0, name, vp_1);
      }

      for (size_t i = 0; i < pairs.size(); ++i) {

        auto p1 = centroids[pairs[i].first];
        auto p2 = centroids[pairs[i].second];
        auto mid = 0.5 * (p1 + p2);

        std::string name = fmt::format("pair_{}", i);
        PointT p;
        p.getArray3fMap() = mid.cast<float>();
        viewer->addText3D(fmt::format("P{}", i), p, 0.2, 1.0, 1.0, 1.0, name,
                          vp_1);
        viewer->addText3D(
            fmt::format("{}", pairs[i].first),
            PointT{static_cast<float>(p1[0]), static_cast<float>(p1[1]),
                   static_cast<float>(p1[2])},
            0.1, 1.0, 1.0, 1.0, fmt::format("text_{}_{}", name, pairs[i].first),
            vp_1);
        viewer->addText3D(
            fmt::format("{}", pairs[i].second),
            PointT{static_cast<float>(p2[0]), static_cast<float>(p2[1]),
                   static_cast<float>(p2[2])},
            0.1, 1.0, 1.0, 1.0,
            fmt::format("text_{}_{}", name, pairs[i].second), vp_1);
      }
    });
  }

  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);

  // INFO: Display floors
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying floors");
      constexpr double offset = 1.0;
      auto map_val = [&min, &max, offset](const double val) {
        const double min_in = min.z - offset;
        const double max_in = max.z + offset;
        constexpr double min_out = 0.0;
        const double max_out = static_cast<double>(pcl::ViridisLUT::size());
        return static_cast<size_t>(
            (val - min_in) / (max_in - min_in) * (max_out - min_out) + min_out);
      };

      for (size_t i = 0; i < horizontal.size(); ++i) {
        const auto id = horizontal[i];
        const double height = centroids[id][2];
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(
            new pcl::PointCloud<pcl::PointXYZ>);
        points->push_back(
            pcl::PointXYZ(min.x - offset, min.y - offset, height));
        points->push_back(
            pcl::PointXYZ(max.x + offset, min.y - offset, height));
        points->push_back(
            pcl::PointXYZ(max.x + offset, max.y + offset, height));
        points->push_back(
            pcl::PointXYZ(min.x - offset, max.y + offset, height));

        pcl::Vertices face;
        face.vertices = {0, 1, 2, 3, 0};

        auto color = pcl::ViridisLUT::at(map_val(height));

        std::string name = fmt::format("floor_{}-layer", i);
        viewer->addPolygonMesh<pcl::PointXYZ>(points, {face}, name, vp_1);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            static_cast<double>(color.r) / 255.0,
            static_cast<double>(color.g) / 255.0,
            static_cast<double>(color.b) / 255.0, name);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name, vp_1);
      }
    });
  }

  auto viz_callback = [&queue_mutex, &task_queue,
                       vp_2](size_t idx,
                             std::vector<std::array<double, 3>> const &pts,
                             std::vector<int> const &indices) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([idx, vp_2, &pts, &indices](VisualizerPtr viewer) {
      auto points = CloudPtr(new Cloud);
      for (const auto &p : pts) {
        PointT pt;
        pt.x = static_cast<float>(p[0]);
        pt.y = static_cast<float>(p[1]);
        pt.z = static_cast<float>(p[2]);
        pt.r = 0;
        pt.g = 0;
        pt.b = 0;
        points->push_back(pt);
      }

      pcl::Vertices face;
      face.vertices = indices;
      auto color = pcl::GlasbeyLUT::at(idx);

      // Add polygon to viewer
      const std::string name = fmt::format("temp_{}", idx);
      viewer->addPolygonMesh<PointT>(points, {face}, name, vp_2);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          static_cast<double>(color.r) / 255.0,
          static_cast<double>(color.g) / 255.0,
          static_cast<double>(color.b) / 255.0, name, vp_2);
      // viewer->setPointCloudRenderingProperties(
      //     pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name, vp_2);
    });
  };

  std::shared_ptr<ReUseX::geometry::CellComplex> cc =
      std::make_shared<ReUseX::geometry::CellComplex>(
          planes, vertical, horizontal, pairs,
          std::array<double, 2>{min.x - 1, min.y - 1},
          std::array<double, 2>{max.x + 1, max.y + 1},
          /*opt.display ? std::optional{viz_callback} : */ std::nullopt);

  spdlog::debug("Cell complex: {}", *cc);
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying cell complex vertices");
      // INFO: Display vertices
      auto vertices = CloudPtr(new Cloud);
      vertices->points.resize(cc->num_vertices());
      for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
        const auto id = (*cc)[*vit].id;
        const auto pos = (*cc)[*vit].pos;
        vertices->points[id].x = pos[0];
        vertices->points[id].y = pos[1];
        vertices->points[id].z = pos[2];
      }
      const std::string v_name = "cellcomplex_vertices";
      pcl::visualization::PointCloudColorHandlerCustom<PointT> v_color_handler(
          vertices, 0, 0, 255);
      viewer->addPointCloud<PointT>(vertices, v_color_handler, v_name, vp_2);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, v_name, vp_2);

      // INFO: Display faces
      spdlog::trace("Displaying cell complex faces");
      auto faces = CloudPtr(new Cloud);
      faces->points.resize(cc->num_faces());
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto id = (*cc)[*fit].id;
        const auto pos = (*cc)[*fit].pos;
        faces->points[id].x = pos[0];
        faces->points[id].y = pos[1];
        faces->points[id].z = pos[2];
      }
      const std::string f_name = "cellcomplex_faces";
      pcl::visualization::PointCloudColorHandlerCustom<PointT> f_color_handler(
          faces, 0, 255, 0);
      viewer->addPointCloud<PointT>(faces, f_color_handler, f_name, vp_2);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, f_name, vp_2);

      // INFO: Display cell centers
      spdlog::trace("Displaying cell complex cell centers");
      auto cells = CloudPtr(new Cloud);
      cells->points.resize(cc->num_cells());
      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto id = (*cc)[*cit].id;
        const auto pos = (*cc)[*cit].pos;
        cells->points[id].x = pos[0];
        cells->points[id].y = pos[1];
        cells->points[id].z = pos[2];
      }
      pcl::visualization::PointCloudColorHandlerCustom<PointT> c_color_handler(
          cells, 255, 0, 0);
      const std::string c_name = "cell_centers";
      viewer->addPointCloud<PointT>(cells, c_color_handler, c_name, vp_2);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, c_name, vp_2);

      // INFO: Display Cell Face Correspondences
      spdlog::trace("Displaying cell face correspondences");
      std::string cf_name = "correspondences_cf";
      pcl::CorrespondencesPtr cf(new pcl::Correspondences);
      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto cid = (*cc)[*cit].id;
        for (auto fit = cc->faces_begin(*cit); fit != cc->faces_end(*cit);
             ++fit) {
          const auto fid = (*cc)[*fit].id;
          cf->emplace_back(static_cast<int>(cid), static_cast<int>(fid), 0.0);
        }
      }
      viewer->addCorrespondences<PointT>(cells, faces, *cf, cf_name, vp_2);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cf_name,
          vp_2);

      // INFO: Display Face Vertex Correspondences
      spdlog::trace("Displaying face vertex correspondences");
      std::string fv_name = "correspondences_fv";
      pcl::CorrespondencesPtr fv(new pcl::Correspondences);
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto fid = (*cc)[*fit].id;
        for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
             ++vit) {
          const auto vid = (*cc)[*vit].id;
          fv->emplace_back(static_cast<int>(fid), static_cast<int>(vid), 0.0);
        }
      }
      viewer->addCorrespondences<PointT>(faces, vertices, *fv, fv_name, vp_2);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, fv_name,
          vp_2);
    });
  }

  cc->compute_room_probabilities<PointT, NormalT, LabelT>(cloud, normals,
                                                          rooms);
  // INFO: Display room probabilities
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying room probabilities");
      auto c_rp = cc->property_map<ReUseX::geometry::CellComplex::Vertex,
                                   std::vector<double>>("c:room_probabilities");
      // Create a Lut for the room labels
      std::vector<pcl::RGB> lut(cc->n_rooms + 1);
      for (size_t i = 0; i < lut.size(); ++i)
        lut[i] = pcl::GlasbeyLUT::at(i);

      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto id = (*cc)[*cit].id;
        const auto &vec = c_rp[*cit];

        // Mix the colors based on the probabilities
        pcl::RGB color{0, 0, 0};
        for (size_t i = 0; i < vec.size(); ++i) {
          const auto c = lut[i];
          color.r += static_cast<uint8_t>(c.r * vec[i]);
          color.g += static_cast<uint8_t>(c.g * vec[i]);
          color.b += static_cast<uint8_t>(c.b * vec[i]);
        }
        // color.r /= static_cast<uint8_t>(vec.size());
        // color.g /= static_cast<uint8_t>(vec.size());
        // color.b /= static_cast<uint8_t>(vec.size());

        std::string name = fmt::format("cell_{}-prob", id);
        PointT p;
        p.x = (*cc)[*cit].pos[0];
        p.y = (*cc)[*cit].pos[1];
        p.z = (*cc)[*cit].pos[2];
        // TODO: Scale sphere size based on if its a room or not
        auto prob_outside = vec[0];
        double r = 0.4 * (1.0 - prob_outside) + 0.1;

        if (prob_outside > 0.9)
          continue; // Skip mostly outside cells

        viewer->addSphere(p, r, static_cast<double>(color.r) / 255.0,
                          static_cast<double>(color.g) / 255.0,
                          static_cast<double>(color.b) / 255.0, name, vp_3);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, name, vp_3);

        // PointT p = cell_centers->points[id];
        // viewer->addText3D(fmt::format("C{}:R{}", id, prob_str), p,
        // 0.05, 1.0, 1.0, 1.0, name);
      }
    });
  }

  cc->compute_face_coverage<PointT>(cloud, planes, inliers);
  // INFO: Display face support probabilities
  if (false) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying face support probabilities");
      auto vertices = CloudPtr(new Cloud);
      vertices->points.resize(cc->num_vertices());
      for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
        const auto id = (*cc)[*vit].id;
        const auto pos = (*cc)[*vit].pos;
        vertices->points[id].x = pos[0];
        vertices->points[id].y = pos[1];
        vertices->points[id].z = pos[2];
      }

      auto f_sp =
          cc->property_map<ReUseX::geometry::CellComplex::Vertex, double>(
              "f:support_probability");
      for (auto fit = cc->faces_begin(); fit != cc->faces_end(); ++fit) {
        const auto id = (*cc)[*fit].id;
        const auto prob = f_sp[*fit] == -1.0 ? 1.0 : f_sp[*fit];

        const auto c = f_sp[*fit] == -1.0
                           ? pcl::RGB(255, 0, 0)
                           : pcl::ViridisLUT::at(static_cast<size_t>(
                                 prob * pcl::ViridisLUT::size()));

        std::string name = fmt::format("face_{}-prob", id);

        // Create face
        pcl::Vertices face{};
        for (auto vit = cc->vertices_begin(*fit); vit != cc->vertices_end(*fit);
             ++vit)
          face.vertices.push_back(static_cast<int>((*cc)[*vit].id));

        // Close the face
        if (!face.vertices.empty())
          face.vertices.push_back(face.vertices[0]);
        else
          continue;

        // std::reverse(face.vertices.begin(), face.vertices.end());
        // ranges::reverse(face.vertices);

        viewer->addPolygonMesh<PointT>(vertices, {face}, name, vp_3);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            static_cast<double>(c.r) / 255.0, static_cast<double>(c.g) / 255.0,
            static_cast<double>(c.b) / 255.0, name, vp_3);

        auto opacity = f_sp[*fit] == -1.0 ? 1.0 : prob * 0.8 + 0.2;
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, name, vp_3);

        if (f_sp[*fit] != -1.0)
          continue; // Only display text for unsupported faces

        auto plane_id = std::get<FaceData>((*cc)[*fit].data).plane_id;

        viewer->addText3D(fmt::format("P{}", plane_id),
                          PointT{static_cast<float>((*cc)[*fit].pos[0]),
                                 static_cast<float>((*cc)[*fit].pos[1]),
                                 static_cast<float>((*cc)[*fit].pos[2])},
                          0.3, 1.0, 1.0, 1.0, fmt::format("text_face_{}", id),
                          vp_3);
      }
    });
  }

  // INFO: Solve the ILP
  spdlog::debug("Cell complex: {}", *cc);

  spdlog::trace("Initializing Solidifier");
  ReUseX::geometry::Solidifier solidifier(cc);
  auto results = solidifier.solve();

  // std::optional<
  //     std::pair<std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
  //     int>,
  //               std::unordered_map<ReUseX::geometry::CellComplex::Vertex,
  //               std::set<int>>>>
  //     results({});

  // INFO: Display results
  if (results.has_value()) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    task_queue.push([&](VisualizerPtr viewer) {
      spdlog::trace("Displaying results");

      // TODO: Make view base constructor form CellComplex
      auto points = CloudPtr(new Cloud);
      points->points.resize(cc->num_vertices());
      for (auto vit = cc->vertices_begin(); vit != cc->vertices_end(); ++vit) {
        const auto id = (*cc)[*vit].id;
        const auto pos = (*cc)[*vit].pos;
        points->points[id].x = pos[0];
        points->points[id].y = pos[1];
        points->points[id].z = pos[2];
      }

      auto [res_room_labels, res_wall_labels] = results.value();

      for (auto cit = cc->cells_begin(); cit != cc->cells_end(); ++cit) {
        const auto id = (*cc)[*cit].id;
        const auto pos = (*cc)[*cit].pos;
        const auto name = fmt::format("cell_{}", id);
        const auto label = res_room_labels[*cit];
        const auto color = pcl::GlasbeyLUT::at(label);

        PointT c;
        c.x = pos[0];
        c.y = pos[1];
        c.z = pos[2];

        switch (label) {
        case 0:
          viewer->addText3D(fmt::format("R{} W[{}]", label,
                                        fmt::join(res_wall_labels[*cit], ",")),
                            c, 0.1, 1.0, 1.0, 1.0, name, vp_4);
          break;
        default:
          std::vector<pcl::Vertices> faces{};
          size_t count = 0;

          for (auto fit = cc->faces_begin(*cit); fit != cc->faces_end(*cit);
               ++fit, ++count) {
            pcl::Vertices face{};

            for (auto vit = cc->vertices_begin(*fit);
                 vit != cc->vertices_end(*fit); ++vit) {
              const auto vid = (*cc)[*vit].id;
              face.vertices.push_back(static_cast<int>(vid));
            }

            // Close the face
            if (!face.vertices.empty())
              face.vertices.push_back(face.vertices[0]);

            faces.push_back(face);
          }
          viewer->addPolygonMesh<PointT>(points, faces, name, vp_4);
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR,
              static_cast<double>(color.r) / 255.0,
              static_cast<double>(color.g) / 255.0,
              static_cast<double>(color.b) / 255.0, name, vp_4);
        }
      }
    });
  }

  viz_thread.join();

  // TODO: Save the output as file

  return 0;
}
