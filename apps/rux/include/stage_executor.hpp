// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/pipeline/stages.hpp>

/// The stage executor rux's in-process job runner (the Qt client, via
/// GuiLaunch) uses: `optimize` through the slam module (which reusex_pipeline
/// does not link, #464), every other stage through
/// pipeline::default_stage_executor().
reusex::pipeline::StageExecutor make_stage_executor();
