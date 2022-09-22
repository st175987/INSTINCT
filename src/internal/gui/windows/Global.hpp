// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Global.hpp
/// @brief Global windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-22

#pragma once

namespace NAV::gui::windows
{

/// @brief Flag whether the ImGui Demo window should be displayed
extern bool showImGuiDemoWindow;
/// @brief Flag whether the ImPlot Demo window should be displayed
extern bool showImPlotDemoWindow;

/// @brief Flag whether the NodeEditor style editor windows should be displayed
extern bool showNodeEditorStyleEditor;
/// @brief Flag whether the ImPlot style editor windows should be displayed
extern bool showImPlotStyleEditor;
/// @brief Flag whether the Font size editor window should be displayed
extern bool showFontSizeEditor;

/// @brief Called every frame to render global windows
void renderGlobalWindows();

} // namespace NAV::gui::windows