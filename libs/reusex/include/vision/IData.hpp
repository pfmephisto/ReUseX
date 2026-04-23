// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace reusex::vision {
/* IData is an interface for data objects in the ReUseX vision module.
 * It is designed to be inherited by specific data types that will implement
 * the necessary functionality for handling vision-related data.
 */
struct IData {
  /* * Virtual destructor to ensure proper cleanup of derived classes.
   */
  virtual ~IData() = default;
};
} // namespace reusex::vision
