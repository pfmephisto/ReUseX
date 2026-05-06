// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

namespace py = pybind11;

PYBIND11_MODULE(_reusex, m) {
  m.doc() = "ReUseX Python bindings — read-only access to .rux project databases";

  using PDB = reusex::ProjectDB;
  using Summary = PDB::ProjectSummary;

  // --- ProjectSummary nested structs (read-only) ---

  py::class_<Summary::ProjectInfo>(m, "ProjectInfo")
      .def_readonly("id", &Summary::ProjectInfo::id)
      .def_readonly("name", &Summary::ProjectInfo::name)
      .def_readonly("building_address", &Summary::ProjectInfo::building_address)
      .def_readonly("year_of_construction",
                    &Summary::ProjectInfo::year_of_construction)
      .def_readonly("survey_date", &Summary::ProjectInfo::survey_date)
      .def_readonly("survey_organisation",
                    &Summary::ProjectInfo::survey_organisation)
      .def_readonly("notes", &Summary::ProjectInfo::notes)
      .def("__repr__", [](const Summary::ProjectInfo &p) {
        return "<ProjectInfo name='" + p.name + "'>";
      });

  py::class_<Summary::CloudInfo>(m, "CloudInfo")
      .def_readonly("name", &Summary::CloudInfo::name)
      .def_readonly("type", &Summary::CloudInfo::type)
      .def_readonly("point_count", &Summary::CloudInfo::point_count)
      .def_readonly("width", &Summary::CloudInfo::width)
      .def_readonly("height", &Summary::CloudInfo::height)
      .def_readonly("organized", &Summary::CloudInfo::organized)
      .def_readonly("labels", &Summary::CloudInfo::labels)
      .def("__repr__", [](const Summary::CloudInfo &c) {
        return "<CloudInfo name='" + c.name + "' type='" + c.type +
               "' points=" + std::to_string(c.point_count) + ">";
      });

  py::class_<Summary::MeshInfo>(m, "MeshInfo")
      .def_readonly("name", &Summary::MeshInfo::name)
      .def_readonly("vertex_count", &Summary::MeshInfo::vertex_count)
      .def_readonly("face_count", &Summary::MeshInfo::face_count)
      .def("__repr__", [](const Summary::MeshInfo &mi) {
        return "<MeshInfo name='" + mi.name +
               "' vertices=" + std::to_string(mi.vertex_count) +
               " faces=" + std::to_string(mi.face_count) + ">";
      });

  py::class_<Summary::SensorFrameInfo>(m, "SensorFrameInfo")
      .def_readonly("total_count", &Summary::SensorFrameInfo::total_count)
      .def_readonly("width", &Summary::SensorFrameInfo::width)
      .def_readonly("height", &Summary::SensorFrameInfo::height)
      .def_readonly("segmented_count",
                    &Summary::SensorFrameInfo::segmented_count)
      .def("__repr__", [](const Summary::SensorFrameInfo &sf) {
        return "<SensorFrameInfo count=" + std::to_string(sf.total_count) +
               " segmented=" + std::to_string(sf.segmented_count) + ">";
      });

  py::class_<Summary::PanoramicInfo>(m, "PanoramicInfo")
      .def_readonly("total_count", &Summary::PanoramicInfo::total_count)
      .def_readonly("matched_count", &Summary::PanoramicInfo::matched_count)
      .def("__repr__", [](const Summary::PanoramicInfo &pi) {
        return "<PanoramicInfo count=" + std::to_string(pi.total_count) +
               " matched=" + std::to_string(pi.matched_count) + ">";
      });

  py::class_<Summary::ComponentInfo>(m, "ComponentInfo")
      .def_readonly("total_count", &Summary::ComponentInfo::total_count)
      .def_readonly("count_by_type", &Summary::ComponentInfo::count_by_type)
      .def("__repr__", [](const Summary::ComponentInfo &ci) {
        return "<ComponentInfo count=" + std::to_string(ci.total_count) + ">";
      });

  py::class_<Summary::MaterialInfo>(m, "MaterialInfo")
      .def_readonly("id", &Summary::MaterialInfo::id)
      .def_readonly("guid", &Summary::MaterialInfo::guid)
      .def_readonly("property_count", &Summary::MaterialInfo::property_count)
      .def_readonly("created_at", &Summary::MaterialInfo::created_at)
      .def_readonly("version_number", &Summary::MaterialInfo::version_number)
      .def("__repr__", [](const Summary::MaterialInfo &mi) {
        return "<MaterialInfo id='" + mi.id + "' guid='" + mi.guid + "'>";
      });

  // --- ProjectSummary ---

  py::class_<Summary>(m, "ProjectSummary")
      .def_readonly("path", &Summary::path)
      .def_readonly("schema_version", &Summary::schema_version)
      .def_readonly("projects", &Summary::projects)
      .def_readonly("clouds", &Summary::clouds)
      .def_readonly("meshes", &Summary::meshes)
      .def_readonly("sensor_frames", &Summary::sensor_frames)
      .def_readonly("panoramic_images", &Summary::panoramic_images)
      .def_readonly("components", &Summary::components)
      .def_readonly("materials", &Summary::materials)
      .def("__repr__", [](const Summary &s) {
        return "<ProjectSummary schema_v" + std::to_string(s.schema_version) +
               " clouds=" + std::to_string(s.clouds.size()) +
               " meshes=" + std::to_string(s.meshes.size()) + ">";
      });

  // --- PipelineLogEntry ---

  py::class_<PDB::PipelineLogEntry>(m, "PipelineLogEntry")
      .def_readonly("id", &PDB::PipelineLogEntry::id)
      .def_readonly("stage", &PDB::PipelineLogEntry::stage)
      .def_readonly("started_at", &PDB::PipelineLogEntry::started_at)
      .def_readonly("finished_at", &PDB::PipelineLogEntry::finished_at)
      .def_readonly("parameters", &PDB::PipelineLogEntry::parameters)
      .def_readonly("status", &PDB::PipelineLogEntry::status)
      .def_readonly("error_msg", &PDB::PipelineLogEntry::error_msg)
      .def("__repr__", [](const PDB::PipelineLogEntry &e) {
        return "<PipelineLogEntry id=" + std::to_string(e.id) + " stage='" +
               e.stage + "' status='" + e.status + "'>";
      });

  // --- ProjectDB ---

  py::class_<PDB>(m, "ProjectDB")
      .def(py::init<std::filesystem::path, bool>(), py::arg("path"),
           py::arg("read_only") = true,
           "Open a ReUseX project database.\n\n"
           "Args:\n"
           "    path: Path to the .rux database file\n"
           "    read_only: Open in read-only mode (default: True)")
      .def("is_open", &PDB::is_open, "Check if the database is open")
      .def("path", &PDB::path, "Get the database file path")
      .def("schema_version", &PDB::schema_version,
           "Get the database schema version")
      .def("project_summary", &PDB::project_summary,
           "Get a summary of all project data")
      .def("list_point_clouds", &PDB::list_point_clouds,
           "List names of all stored point clouds")
      .def("list_meshes", &PDB::list_meshes,
           "List names of all stored meshes")
      .def("list_building_components",
           py::overload_cast<>(&PDB::list_building_components, py::const_),
           "List names of all building components")
      .def("building_component_count", &PDB::building_component_count,
           "Get the total number of building components")
      .def("pipeline_log", &PDB::pipeline_log, py::arg("limit") = 0,
           "Get pipeline execution log entries.\n\n"
           "Args:\n"
           "    limit: Max entries to return (0 = all)")

      // --- Point cloud geometry loading (returns numpy arrays) ---

      .def(
          "point_cloud_xyzrgb",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_xyzrgb(name);
            const ssize_t n =
                cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto positions = py::array_t<float>({n, ssize_t{3}});
            auto colors = py::array_t<uint8_t>({n, ssize_t{3}});

            if (n > 0) {
              auto pos = positions.mutable_unchecked<2>();
              auto col = colors.mutable_unchecked<2>();
              for (ssize_t i = 0; i < n; ++i) {
                const auto &pt = (*cloud)[static_cast<size_t>(i)];
                pos(i, 0) = pt.x;
                pos(i, 1) = pt.y;
                pos(i, 2) = pt.z;
                col(i, 0) = pt.r;
                col(i, 1) = pt.g;
                col(i, 2) = pt.b;
              }
            }

            py::dict result;
            result["positions"] = positions;
            result["colors"] = colors;
            return result;
          },
          py::arg("name"),
          "Load a PointXYZRGB cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'positions': (N, 3) float32\n"
          "    'colors':    (N, 3) uint8 (RGB)")

      .def(
          "point_cloud_xyz",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_xyz(name);
            const ssize_t n =
                cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto positions = py::array_t<float>({n, ssize_t{3}});

            if (n > 0) {
              auto pos = positions.mutable_unchecked<2>();
              for (ssize_t i = 0; i < n; ++i) {
                const auto &pt = (*cloud)[static_cast<size_t>(i)];
                pos(i, 0) = pt.x;
                pos(i, 1) = pt.y;
                pos(i, 2) = pt.z;
              }
            }

            py::dict result;
            result["positions"] = positions;
            return result;
          },
          py::arg("name"),
          "Load a PointXYZ cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'positions': (N, 3) float32")

      .def(
          "point_cloud_label",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_label(name);
            const ssize_t n =
                cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto labels = py::array_t<int32_t>(n);

            if (n > 0) {
              auto lbl = labels.mutable_unchecked<1>();
              for (ssize_t i = 0; i < n; ++i)
                lbl(i) =
                    static_cast<int32_t>((*cloud)[static_cast<size_t>(i)].label);
            }

            py::dict result;
            result["labels"] = labels;
            return result;
          },
          py::arg("name"),
          "Load a Label cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'labels': (N,) int32 (label IDs per point)")

      .def("label_definitions", &PDB::label_definitions, py::arg("name"),
           "Get label ID → name definitions for a cloud.\n\n"
           "Returns dict mapping int label_id to str label_name.")

      .def("__repr__", [](const PDB &db) {
        return "<ProjectDB path='" + db.path().string() +
               "' open=" + (db.is_open() ? "True" : "False") + ">";
      });
}
