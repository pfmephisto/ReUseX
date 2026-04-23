// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <string>
#include <vector>

namespace reusex::vision {

/**
 * @brief YOLO v8 class names for object detection.
 *
 * Contains the list of class names that can be detected by YOLO v8 model.
 */
extern std::vector<std::string> Yolov8_className;

} // namespace reusex::vision
