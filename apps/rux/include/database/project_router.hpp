// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for project metadata
 *
 * Handles paths like:
 * - projects → list all projects (JSON array)
 * - projects[0] → get first project (JSON object)
 * - projects[0].name → get project name (text)
 * - project → alias for projects[0] (convenience)
 * - project.name → alias for projects[0].name
 *
 * Note: "projects" is a collection (array), while "project" is a
 * convenience alias for the first/only project.
 */
class ProjectRouter : public ResourceRouter {
public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;

private:
  nlohmann::json get_project_json(std::string_view project_id) const;
};

} // namespace rux::database
