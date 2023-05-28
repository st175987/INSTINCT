// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ErrorModel.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Eigen.hpp"

#include <imgui_internal.h>
#include <limits>

// ---------------------------------------------------------- Private variables ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), PosVelAtt::type() };

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] rng Object to read info from
void to_json(json& j, const ErrorModel::RandomNumberGenerator& rng)
{
    j = json{
        { "useSeedInsteadOfSystemTime", rng.useSeedInsteadOfSystemTime },
        { "seed", rng.seed },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] rng Output object
void from_json(const json& j, ErrorModel::RandomNumberGenerator& rng)
{
    if (j.contains("useSeedInsteadOfSystemTime"))
    {
        j.at("useSeedInsteadOfSystemTime").get_to(rng.useSeedInsteadOfSystemTime);
    }
    if (j.contains("seed"))
    {
        j.at("seed").get_to(rng.seed);
    }
}

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::ErrorModel::ErrorModel()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 812, 332 };

    nm::CreateInputPin(this, "True", Pin::Type::Flow, supportedDataIdentifier, &ErrorModel::receiveObs);

    nm::CreateOutputPin(this, "Biased", Pin::Type::Flow, supportedDataIdentifier);
}

NAV::ErrorModel::~ErrorModel()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ErrorModel::typeStatic()
{
    return "ErrorModel";
}

std::string NAV::ErrorModel::type() const
{
    return typeStatic();
}

std::string NAV::ErrorModel::category()
{
    return "Data Processor";
}

void NAV::ErrorModel::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    float itemWidth = 470 * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 180 * gui::NodeEditorApplication::windowFontRatio();

    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        if (ImGui::BeginTabBar("ImuErrorTabBar", ImGuiTabBarFlags_None))
        {
            if (ImGui::BeginTabItem("Accelerometer"))
            {
                ImGui::TextUnformatted("Offsets:");
                ImGui::Indent();

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias (platform)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                                       _imuAccelerometerBias_p.data(), reinterpret_cast<int*>(&_imuAccelerometerBiasUnit), "m/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerBias_p changed to {}", nameId(), _imuAccelerometerBias_p.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerBiasUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerBiasUnit));
                    flow::ApplyChanges();
                }

                ImGui::Unindent();
                ImGui::TextUnformatted("Measurement noise:");
                ImGui::Indent();

                if (gui::widgets::InputDouble3WithUnit(fmt::format("f⁰ Noise ({})##{}",
                                                                   _imuAccelerometerNoiseUnit == ImuAccelerometerNoiseUnits::m_s2 ? "Standard deviation"
                                                                                                                                  : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerWhiteNoiseInput.data(), reinterpret_cast<int*>(&_imuAccelerometerNoiseUnit), "m/s^2\0m^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerWhiteNoiseInput changed to {}", nameId(), _imuAccelerometerWhiteNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerNoiseUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("S(f)~f⁰ Noise, also called White Noise or Velocity Random Walk\n"
                                           "Adds normally distributed random numbers\n"
                                           "with given standard deviation/variance to accelerometer data.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("f⁻² Noise ({})##{}",
                                                                   _imuAccelerometerNoiseUnit == ImuAccelerometerNoiseUnits::m_s2 ? "Standard deviation"
                                                                                                                                  : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerRedNoiseInput.data(), reinterpret_cast<int*>(&_imuAccelerometerNoiseUnit), "m/s^2\0m^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerRedNoiseInput changed to {}", nameId(), _imuAccelerometerRedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerNoiseUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("S(f)~f⁻² Noise, also called Red Noise or Rate Random Walk\n"
                                           "The variance at time t equals (t-t₀)σ².");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise ({})##{}",
                                                                   _imuAccelerometerNoiseUnit == ImuAccelerometerNoiseUnits::m_s2 ? "Standard deviation"
                                                                                                                                  : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerCorrelatedNoiseInput.data(), reinterpret_cast<int*>(&_imuAccelerometerNoiseUnit), "m/s^2\0m^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerCorrelatedNoiseInput changed to {}", nameId(), _imuAccelerometerCorrelatedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerNoiseUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Standard deviation/Variance of Correlated Noise\n"
                                           "Also called 1st order Gauss-Markov process.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise Correlation Time ##{}", size_t(id)).c_str(),
                                                       itemWidth, unitWidth, _imuAccelerometerCorrelatedNoiseCorrelationTime.data(), reinterpret_cast<int*>(&_imuAccelerometerCorrelationTimeUnit), "s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerCorrelatedNoiseCorrelationTime changed to {}", nameId(), _imuAccelerometerCorrelatedNoiseCorrelationTime.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerCorrelationTimeUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerCorrelationTimeUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Correlation time of Correlated Noise\n"
                                           "Also called 1st order Gauss-Markov process.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                float currentCursorX = ImGui::GetCursorPosX();
                if (ImGui::Checkbox(fmt::format("##_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime))
                {
                    LOG_DEBUG("{}: _imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime);
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (!_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime)
                {
                    ImGui::BeginDisabled();
                }
                ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
                if (ImGui::SliderUInt(fmt::format("Noise Seed##{}", size_t(id)).c_str(), &_imuAccelerometerRandomNumberGenerator.seed, 0, std::numeric_limits<uint32_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
                {
                    LOG_DEBUG("{}: _imuAccelerometerRandomNumberGenerator.seed changed to {}", nameId(), _imuAccelerometerRandomNumberGenerator.seed);
                    flow::ApplyChanges();
                }
                if (!_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime)
                {
                    ImGui::EndDisabled();
                }
                ImGui::Unindent();

                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Gyroscope"))
            {
                ImGui::TextUnformatted("Offsets:");
                ImGui::Indent();

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias (platform)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                                       _imuGyroscopeBias_p.data(), reinterpret_cast<int*>(&_imuGyroscopeBiasUnit), "rad/s\0deg/s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeBias_p changed to {}", nameId(), _imuGyroscopeBias_p.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeBiasUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeBiasUnit));
                    flow::ApplyChanges();
                }

                ImGui::Unindent();
                ImGui::TextUnformatted("Measurement noise:");
                ImGui::Indent();

                float currentCursorX = ImGui::GetCursorPosX();

                if (gui::widgets::InputDouble3WithUnit(fmt::format("f⁰ Noise ({})##{}",
                                                                   _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::rad_s
                                                                           || _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::deg_s
                                                                       ? "Standard deviation"
                                                                       : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeWhiteNoiseInput.data(), reinterpret_cast<int*>(&_imuGyroscopeNoiseUnit), "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeWhiteNoiseInput changed to {}", nameId(), _imuGyroscopeWhiteNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeNoiseUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("S(f)~f⁰ Noise, also called White Noise or Angle Random Walk\n"
                                           "Adds normally distributed random numbers\n"
                                           "with given standard deviation/variance to gyroscope data.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("f⁻² Noise ({})##{}",
                                                                   _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::rad_s
                                                                           || _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::deg_s
                                                                       ? "Standard deviation"
                                                                       : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeRedNoiseInput.data(), reinterpret_cast<int*>(&_imuGyroscopeNoiseUnit), "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeRedNoiseInput changed to {}", nameId(), _imuGyroscopeRedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeNoiseUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("S(f)~f⁻² Noise, also called Red Noise or Rate Random Walk\n"
                                           "The variance at time t equals (t-t₀)σ².");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise ({})##{}",
                                                                   _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::rad_s
                                                                           || _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::deg_s
                                                                       ? "Standard deviation"
                                                                       : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeCorrelatedNoiseInput.data(), reinterpret_cast<int*>(&_imuGyroscopeNoiseUnit), "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeCorrelatedNoiseInput changed to {}", nameId(), _imuGyroscopeCorrelatedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeNoiseUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Standard deviation/Variance of Correlated Noise\n"
                                           "Also called 1st order Gauss-Markov process.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise Correlation Time ##{}", size_t(id)).c_str(),
                                                       itemWidth, unitWidth, _imuGyroscopeCorrelatedNoiseCorrelationTime.data(), reinterpret_cast<int*>(&_imuGyroscopeCorrelationTimeUnit), "s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeCorrelatedNoiseCorrelationTime changed to {}", nameId(), _imuGyroscopeCorrelatedNoiseCorrelationTime.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeCorrelationTimeUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeCorrelationTimeUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Correlation time of Correlated Noise\n"
                                           "Also called 1st order Gauss-Markov process.");
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
                currentCursorX = ImGui::GetCursorPosX();
                if (ImGui::Checkbox(fmt::format("##_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime))
                {
                    LOG_DEBUG("{}: _imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime);
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (!_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime)
                {
                    ImGui::BeginDisabled();
                }
                ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
                if (ImGui::SliderUInt(fmt::format("Noise Seed##{}", size_t(id)).c_str(), &(_imuGyroscopeRandomNumberGenerator.seed), 0, std::numeric_limits<uint32_t>::max() / 2UL, "%lu", ImGuiSliderFlags_Logarithmic))
                {
                    LOG_DEBUG("{}: _imuGyroscopeRandomNumberGenerator.seed changed to {}", nameId(), _imuGyroscopeRandomNumberGenerator.seed);
                    flow::ApplyChanges();
                }
                if (!_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime)
                {
                    ImGui::EndDisabled();
                }
                ImGui::Unindent();

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        ImGui::TextUnformatted("Offsets:");
        ImGui::Indent();

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position Bias ({})##{}",
                                                           _positionBiasUnit == PositionBiasUnits::meter ? "NED" : "LatLonAlt",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _positionBias.data(), reinterpret_cast<int*>(&_positionBiasUnit), "[m m m]\0[rad, rad, m]\0[deg, deg, m]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _positionBias changed to {}", nameId(), _positionBias.transpose());
            LOG_DEBUG("{}: _positionBiasUnit changed to {}", nameId(), fmt::underlying(_positionBiasUnit));
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity Bias (NED)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _velocityBias.data(), reinterpret_cast<int*>(&_velocityBiasUnit), "m/s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _velocityBias changed to {}", nameId(), _velocityBias.transpose());
            LOG_DEBUG("{}: _velocityBiasUnit changed to {}", nameId(), fmt::underlying(_velocityBiasUnit));
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("RollPitchYaw Bias##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _attitudeBias.data(), reinterpret_cast<int*>(&_attitudeBiasUnit), "rad\0deg\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _attitudeBias changed to {}", nameId(), _attitudeBias.transpose());
            LOG_DEBUG("{}: _attitudeBiasUnit changed to {}", nameId(), fmt::underlying(_attitudeBiasUnit));
            flow::ApplyChanges();
        }

        ImGui::Unindent();
        ImGui::TextUnformatted("Measurement noise:");
        ImGui::Indent();

        // #########################################################################################################################################
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position Noise ({})##{}",
                                                           _positionNoiseUnit == PositionNoiseUnits::meter
                                                                   || _positionNoiseUnit == PositionNoiseUnits::rad_rad_m
                                                                   || _positionNoiseUnit == PositionNoiseUnits::deg_deg_m
                                                               ? "Standard deviation"
                                                               : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _positionNoise.data(), reinterpret_cast<int*>(&_positionNoiseUnit), "[m m m]\0[rad, rad, m]\0[deg, deg, m]\0[m^2 m^2 m^2]\0[rad^2, rad^2, m^2]\0[deg^2, deg^2, m^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _positionNoise changed to {}", nameId(), _positionNoise.transpose());
            LOG_DEBUG("{}: _positionNoiseUnit changed to {}", nameId(), fmt::underlying(_positionNoiseUnit));
            flow::ApplyChanges();
        }
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_positionRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_positionRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _positionRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _positionRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_positionRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderUInt(fmt::format("Position Noise Seed##{}", size_t(id)).c_str(), &_positionRandomNumberGenerator.seed, 0, std::numeric_limits<uint32_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _positionRandomNumberGenerator.seed changed to {}", nameId(), _positionRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_positionRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::EndDisabled();
        }

        // #########################################################################################################################################

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity Noise ({})##{}",
                                                           _velocityNoiseUnit == VelocityNoiseUnits::m_s ? "Standard deviation"
                                                                                                         : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _velocityNoise.data(), reinterpret_cast<int*>(&_velocityNoiseUnit), "[m/s]\0[m^2/s^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _velocityNoise changed to {}", nameId(), _velocityNoise.transpose());
            LOG_DEBUG("{}: _velocityNoiseUnit changed to {}", nameId(), fmt::underlying(_velocityNoiseUnit));
            flow::ApplyChanges();
        }
        currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _velocityRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _velocityRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderUInt(fmt::format("Velocity Noise Seed##{}", size_t(id)).c_str(), &_velocityRandomNumberGenerator.seed, 0, std::numeric_limits<uint32_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _velocityRandomNumberGenerator.seed changed to {}", nameId(), _velocityRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::EndDisabled();
        }

        // #########################################################################################################################################

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Attitude Noise ({})##{}",
                                                           _attitudeNoiseUnit == AttitudeNoiseUnits::rad
                                                                   || _attitudeNoiseUnit == AttitudeNoiseUnits::deg
                                                               ? "Standard deviation"
                                                               : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _attitudeNoise.data(), reinterpret_cast<int*>(&_attitudeNoiseUnit), "[rad]\0[deg]\0[rad^2]\0[deg^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _attitudeNoise changed to {}", nameId(), _attitudeNoise.transpose());
            LOG_DEBUG("{}: _attitudeNoiseUnit changed to {}", nameId(), fmt::underlying(_attitudeNoiseUnit));
            flow::ApplyChanges();
        }
        currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderUInt(fmt::format("Attitude Noise Seed##{}", size_t(id)).c_str(), &_attitudeRandomNumberGenerator.seed, 0, std::numeric_limits<uint32_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _attitudeRandomNumberGenerator.seed changed to {}", nameId(), _attitudeRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::EndDisabled();
        }
        // #########################################################################################################################################

        ImGui::Unindent();
    }
}

json NAV::ErrorModel::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuAccelerometerBiasUnit"] = _imuAccelerometerBiasUnit;
    j["imuAccelerometerBias_p"] = _imuAccelerometerBias_p;
    j["imuGyroscopeBiasUnit"] = _imuGyroscopeBiasUnit;
    j["imuGyroscopeBias_p"] = _imuGyroscopeBias_p;

    j["imuAccelerometerNoiseUnit"] = _imuAccelerometerNoiseUnit;
    j["imuAccelerometerWhiteNoise"] = _imuAccelerometerWhiteNoiseInput;
    j["imuAccelerometerRedNoise"] = _imuAccelerometerRedNoiseInput;
    j["imuAccelerometerCorrelatedNoise"] = _imuAccelerometerCorrelatedNoiseInput;
    j["imuAccelerometerCorrelatedNoiseCorrelationTime"] = _imuAccelerometerCorrelatedNoiseCorrelationTime;
    j["imuAccelerometerCorrelationTimeUnit"] = _imuAccelerometerCorrelationTimeUnit;
    j["imuAccelerometerRandomNumberGenerator"] = _imuAccelerometerRandomNumberGenerator;
    j["imuGyroscopeNoiseUnit"] = _imuGyroscopeNoiseUnit;
    j["imuGyroscopeWhiteNoise"] = _imuGyroscopeWhiteNoiseInput;
    j["imuGyroscopeRedNoise"] = _imuGyroscopeRedNoiseInput;
    j["imuGyroscopeCorrelatedNoise"] = _imuGyroscopeCorrelatedNoiseInput;
    j["imuGyroscopeCorrelatedNoiseCorrelationTime"] = _imuGyroscopeCorrelatedNoiseCorrelationTime;
    j["imuGyroscopeCorrelationTimeUnit"] = _imuGyroscopeCorrelationTimeUnit;
    j["imuGyroscopeRandomNumberGenerator"] = _imuGyroscopeRandomNumberGenerator;
    // #########################################################################################################################################
    j["positionBiasUnit"] = _positionBiasUnit;
    j["positionBias"] = _positionBias;
    j["velocityBiasUnit"] = _velocityBiasUnit;
    j["velocityBias"] = _velocityBias;
    j["attitudeBiasUnit"] = _attitudeBiasUnit;
    j["attitudeBias"] = _attitudeBias;

    j["positionNoiseUnit"] = _positionNoiseUnit;
    j["positionNoise"] = _positionNoise;
    j["positionRandomNumberGenerator"] = _positionRandomNumberGenerator;
    j["velocityNoiseUnit"] = _velocityNoiseUnit;
    j["velocityNoise"] = _velocityNoise;
    j["velocityRandomNumberGenerator"] = _velocityRandomNumberGenerator;
    j["attitudeNoiseUnit"] = _attitudeNoiseUnit;
    j["attitudeNoise"] = _attitudeNoise;
    j["attitudeRandomNumberGenerator"] = _attitudeRandomNumberGenerator;

    return j;
}

void NAV::ErrorModel::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuAccelerometerBiasUnit"))
    {
        j.at("imuAccelerometerBiasUnit").get_to(_imuAccelerometerBiasUnit);
    }
    if (j.contains("imuAccelerometerBias_p"))
    {
        j.at("imuAccelerometerBias_p").get_to(_imuAccelerometerBias_p);
    }
    if (j.contains("imuGyroscopeBiasUnit"))
    {
        j.at("imuGyroscopeBiasUnit").get_to(_imuGyroscopeBiasUnit);
    }
    if (j.contains("imuGyroscopeBias_p"))
    {
        j.at("imuGyroscopeBias_p").get_to(_imuGyroscopeBias_p);
    }

    if (j.contains("imuAccelerometerNoiseUnit"))
    {
        j.at("imuAccelerometerNoiseUnit").get_to(_imuAccelerometerNoiseUnit);
    }
    if (j.contains("imuAccelerometerWhiteNoise"))
    {
        j.at("imuAccelerometerWhiteNoise").get_to(_imuAccelerometerWhiteNoiseInput);
    }
    if (j.contains("imuAccelerometerRedNoise"))
    {
        j.at("imuAccelerometerRedNoise").get_to(_imuAccelerometerRedNoiseInput);
    }
    if (j.contains("imuAccelerometerCorrelatedNoise"))
    {
        j.at("imuAccelerometerCorrelatedNoise").get_to(_imuAccelerometerCorrelatedNoiseInput);
    }
    if (j.contains("imuAccelerometerCorrelatedNoiseCorrelationTime"))
    {
        j.at("imuAccelerometerCorrelatedNoiseCorrelationTime").get_to(_imuAccelerometerCorrelatedNoiseCorrelationTime);
    }
    if (j.contains("imuAccelerometerCorrelationTimeUnit"))
    {
        j.at("imuAccelerometerCorrelationTimeUnit").get_to(_imuAccelerometerCorrelationTimeUnit);
    }
    if (j.contains("imuAccelerometerRandomNumberGenerator"))
    {
        j.at("imuAccelerometerRandomNumberGenerator").get_to(_imuAccelerometerRandomNumberGenerator);
    }
    if (j.contains("imuGyroscopeNoiseUnit"))
    {
        j.at("imuGyroscopeNoiseUnit").get_to(_imuGyroscopeNoiseUnit);
    }
    if (j.contains("imuGyroscopeWhiteNoise"))
    {
        j.at("imuGyroscopeWhiteNoise").get_to(_imuGyroscopeWhiteNoiseInput);
    }
    if (j.contains("imuGyroscopeRedNoise"))
    {
        j.at("imuGyroscopeRedNoise").get_to(_imuGyroscopeRedNoiseInput);
    }
    if (j.contains("imuGyroscopeCorrelatedNoise"))
    {
        j.at("imuGyroscopeCorrelatedNoise").get_to(_imuGyroscopeCorrelatedNoiseInput);
    }
    if (j.contains("imuGyroscopeCorrelatedNoiseCorrelationTime"))
    {
        j.at("imuGyroscopeCorrelatedNoiseCorrelationTime").get_to(_imuGyroscopeCorrelatedNoiseCorrelationTime);
    }
    if (j.contains("imuGyroscopeCorrelationTimeUnit"))
    {
        j.at("imuGyroscopeCorrelationTimeUnit").get_to(_imuGyroscopeCorrelationTimeUnit);
    }
    if (j.contains("imuGyroscopeRandomNumberGenerator"))
    {
        j.at("imuGyroscopeRandomNumberGenerator").get_to(_imuGyroscopeRandomNumberGenerator);
    }
    // #########################################################################################################################################
    if (j.contains("positionBiasUnit"))
    {
        j.at("positionBiasUnit").get_to(_positionBiasUnit);
    }
    if (j.contains("positionBias"))
    {
        j.at("positionBias").get_to(_positionBias);
    }
    if (j.contains("velocityBiasUnit"))
    {
        j.at("velocityBiasUnit").get_to(_velocityBiasUnit);
    }
    if (j.contains("velocityBias"))
    {
        j.at("velocityBias").get_to(_velocityBias);
    }
    if (j.contains("attitudeBiasUnit"))
    {
        j.at("attitudeBiasUnit").get_to(_attitudeBiasUnit);
    }
    if (j.contains("attitudeBias"))
    {
        j.at("attitudeBias").get_to(_attitudeBias);
    }

    if (j.contains("positionNoiseUnit"))
    {
        j.at("positionNoiseUnit").get_to(_positionNoiseUnit);
    }
    if (j.contains("positionNoise"))
    {
        j.at("positionNoise").get_to(_positionNoise);
    }
    if (j.contains("positionRandomNumberGenerator"))
    {
        j.at("positionRandomNumberGenerator").get_to(_positionRandomNumberGenerator);
    }
    if (j.contains("velocityNoiseUnit"))
    {
        j.at("velocityNoiseUnit").get_to(_velocityNoiseUnit);
    }
    if (j.contains("velocityNoise"))
    {
        j.at("velocityNoise").get_to(_velocityNoise);
    }
    if (j.contains("velocityRandomNumberGenerator"))
    {
        j.at("velocityRandomNumberGenerator").get_to(_velocityRandomNumberGenerator);
    }
    if (j.contains("attitudeNoiseUnit"))
    {
        j.at("attitudeNoiseUnit").get_to(_attitudeNoiseUnit);
    }
    if (j.contains("attitudeNoise"))
    {
        j.at("attitudeNoise").get_to(_attitudeNoise);
    }
    if (j.contains("attitudeRandomNumberGenerator"))
    {
        j.at("attitudeRandomNumberGenerator").get_to(_attitudeRandomNumberGenerator);
    }
}

bool NAV::ErrorModel::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        _imuAccelerometerRandomNumberGenerator.generator.seed(_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                                  ? _imuAccelerometerRandomNumberGenerator.seed
                                                                  : static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _imuGyroscopeRandomNumberGenerator.generator.seed(_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                              ? _imuGyroscopeRandomNumberGenerator.seed
                                                              : static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        _positionRandomNumberGenerator.generator.seed(_positionRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _positionRandomNumberGenerator.seed
                                                          : static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _velocityRandomNumberGenerator.generator.seed(_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _velocityRandomNumberGenerator.seed
                                                          : static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _attitudeRandomNumberGenerator.generator.seed(_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _attitudeRandomNumberGenerator.seed
                                                          : static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    }

    _dt = 0;
    _previousInsTime.reset();

    _imuAccelerometerRedNoise = Eigen::Vector3d::Zero();
    _imuGyroscopeRedNoise = Eigen::Vector3d::Zero();

    _imuAccelerometerCorrelatedNoise = Eigen::Vector3d::Zero();
    _imuGyroscopeCorrelatedNoise = Eigen::Vector3d::Zero();

    return true;
}

void NAV::ErrorModel::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id)
    {
        return; // Link on Output Port
    }

    // Store previous output pin identifier
    auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
    // Overwrite output pin identifier with input pin identifier
    outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin.dataIdentifier;

    if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
    {
        // Check if connected links on output port are still valid
        for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
        {
            if (auto* endPin = link.getConnectedPin())
            {
                if (!outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
                {
                    // If the link is not valid anymore, delete it
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
                }
            }
        }

        // Refresh all links connected to the output pin if the type changed
        if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
        {
            for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
            {
                if (auto* connectedPin = link.getConnectedPin())
                {
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).recreateLink(*connectedPin);
                }
            }
        }
    }
}

void NAV::ErrorModel::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if ((endPin.parentNode->id != id                                  // Link on Output port is removed
         && !inputPins.at(INPUT_PORT_INDEX_FLOW).isPinLinked())       //     and the Input port is not linked
        || (startPin.parentNode->id != id                             // Link on Input port is removed
            && !outputPins.at(OUTPUT_PORT_INDEX_FLOW).isPinLinked())) //     and the Output port is not linked
    {
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = supportedDataIdentifier;
    }
}

void NAV::ErrorModel::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Select the correct data type and make a copy of the node data to modify
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(queue.extract_front())));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(queue.extract_front())));
    }
}

void NAV::ErrorModel::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    if (_previousInsTime.empty())
    {
        _previousInsTime = imuObs->insTime;
    }
    else
    {
        _dt = (imuObs->insTime - _previousInsTime).count();
        _previousInsTime = imuObs->insTime;
    }

    // #########################################################################################################################################

    // Accelerometer Bias in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerBias_p = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerBiasUnit)
    {
    case ImuAccelerometerBiasUnits::m_s2:
        accelerometerBias_p = _imuAccelerometerBias_p;
        break;
    }
    LOG_DATA("{}: accelerometerBias_p = {} [m/s^2]", nameId(), accelerometerBias_p.transpose());

    // Gyroscope Bias in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeBias_p = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeBiasUnit)
    {
    case ImuGyroscopeBiasUnits::deg_s:
        gyroscopeBias_p = deg2rad(_imuGyroscopeBias_p);
        break;
    case ImuGyroscopeBiasUnits::rad_s:
        gyroscopeBias_p = _imuGyroscopeBias_p;
        break;
    }
    LOG_DATA("{}: gyroscopeBias_p = {} [rad/s]", nameId(), gyroscopeBias_p.transpose());

    // #########################################################################################################################################

    // Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerNoiseStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelerometerRedNoiseStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelerometerCorrelatedNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerNoiseUnit)
    {
    case ImuAccelerometerNoiseUnits::m_s2:
        accelerometerNoiseStd = _imuAccelerometerWhiteNoiseInput;
        accelerometerRedNoiseStd = _imuAccelerometerRedNoiseInput;
        accelerometerCorrelatedNoiseStd = _imuAccelerometerCorrelatedNoiseInput;
        break;
    case ImuAccelerometerNoiseUnits::m2_s4:
        accelerometerNoiseStd = _imuAccelerometerWhiteNoiseInput.cwiseSqrt();
        accelerometerRedNoiseStd = _imuAccelerometerRedNoiseInput.cwiseSqrt();
        accelerometerCorrelatedNoiseStd = _imuAccelerometerCorrelatedNoiseInput.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: accelerometerNoiseStd = {} [m/s^2]", nameId(), accelerometerNoiseStd.transpose());
    LOG_DATA("{}: accelerometerRedNoiseStd = {} [m/s^2]", nameId(), accelerometerRedNoiseStd.transpose());
    LOG_DATA("{}: accelerometerCorrelatedNoiseStd = {} [m/s^2]", nameId(), accelerometerCorrelatedNoiseStd.transpose());

    LOG_DATA("{}: accelerometerCorrelatedNoiseCorrelationTime = {} [s]", nameId(), _imuAccelerometerCorrelatedNoiseCorrelationTime.transpose());

    Eigen::Array3d accelerometerCorrelatedNoiseMultiplicationFactor;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        accelerometerCorrelatedNoiseMultiplicationFactor(i) = _imuAccelerometerCorrelatedNoiseCorrelationTime(i) == 0.0 ? 0.0 : static_cast<double>(std::exp(-_dt / std::fabs(_imuAccelerometerCorrelatedNoiseCorrelationTime(i))));
    }

    // Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeNoiseStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeRedNoiseStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeCorrelatedNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeNoiseUnit)
    {
    case ImuGyroscopeNoiseUnits::rad_s:
        gyroscopeNoiseStd = _imuGyroscopeWhiteNoiseInput;
        gyroscopeRedNoiseStd = _imuGyroscopeRedNoiseInput;
        gyroscopeCorrelatedNoiseStd = _imuGyroscopeCorrelatedNoiseInput;
        break;
    case ImuGyroscopeNoiseUnits::deg_s:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeWhiteNoiseInput);
        gyroscopeRedNoiseStd = deg2rad(_imuGyroscopeRedNoiseInput);
        gyroscopeCorrelatedNoiseStd = deg2rad(_imuGyroscopeCorrelatedNoiseInput);
        break;
    case ImuGyroscopeNoiseUnits::rad2_s2:
        gyroscopeNoiseStd = _imuGyroscopeWhiteNoiseInput.cwiseSqrt();
        gyroscopeRedNoiseStd = _imuGyroscopeRedNoiseInput.cwiseSqrt();
        gyroscopeCorrelatedNoiseStd = _imuGyroscopeCorrelatedNoiseInput.cwiseSqrt();
        break;
    case ImuGyroscopeNoiseUnits::deg2_s2:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeWhiteNoiseInput.cwiseSqrt());
        gyroscopeRedNoiseStd = deg2rad(_imuGyroscopeRedNoiseInput.cwiseSqrt());
        gyroscopeCorrelatedNoiseStd = deg2rad(_imuGyroscopeCorrelatedNoiseInput.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: gyroscopeNoiseStd = {} [rad/s]", nameId(), gyroscopeNoiseStd.transpose());
    LOG_DATA("{}: gyroscopeRedNoiseStd = {} [rad/s]", nameId(), gyroscopeRedNoiseStd.transpose());
    LOG_DATA("{}: gyroscopeCorrelatedNoiseStd = {} [rad/s]", nameId(), gyroscopeCorrelatedNoiseStd.transpose());

    LOG_DATA("{}: gyroscopeCorrelatedNoiseCorrelationTIme = {} [rad/s]", nameId(), _imuGyroscopeCorrelatedNoiseCorrelationTime.transpose());

    Eigen::Array3d gyroscopeCorrelatedNoiseMultiplicationFactor;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        gyroscopeCorrelatedNoiseMultiplicationFactor(i) = _imuGyroscopeCorrelatedNoiseCorrelationTime(i) == 0.0 ? 0.0 : static_cast<double>(std::exp(-_dt / std::fabs(_imuGyroscopeCorrelatedNoiseCorrelationTime(i))));
    }

    // #########################################################################################################################################

    _imuAccelerometerRedNoise += sqrt(_dt)
                                 * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerRedNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                    std::normal_distribution<double>{ 0.0, accelerometerRedNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                    std::normal_distribution<double>{ 0.0, accelerometerRedNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) };

    _imuGyroscopeRedNoise += sqrt(_dt)
                             * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeRedNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                std::normal_distribution<double>{ 0.0, gyroscopeRedNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                std::normal_distribution<double>{ 0.0, gyroscopeRedNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) };

    _imuAccelerometerCorrelatedNoise = (_imuAccelerometerCorrelatedNoise.array() * accelerometerCorrelatedNoiseMultiplicationFactor).matrix()
                                       + sqrt(_dt)
                                             * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                                std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                                std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) };

    _imuGyroscopeCorrelatedNoise = (_imuGyroscopeCorrelatedNoise.array() * gyroscopeCorrelatedNoiseMultiplicationFactor).matrix()
                                   + sqrt(_dt)
                                         * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) };

    imuObs->accelUncompXYZ.value() += accelerometerBias_p
                                      + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) }
                                      + _imuAccelerometerRedNoise
                                      + _imuAccelerometerCorrelatedNoise;

    imuObs->gyroUncompXYZ.value() += gyroscopeBias_p
                                     + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) }
                                     + _imuGyroscopeRedNoise
                                     + _imuGyroscopeCorrelatedNoise;

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, imuObs);
}

void NAV::ErrorModel::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    // Position Bias in latLonAlt in [rad, rad, m]
    Eigen::Vector3d lla_positionBias = Eigen::Vector3d::Zero();
    switch (_positionBiasUnit)
    {
    case PositionBiasUnits::meter:
    {
        Eigen::Vector3d e_positionBias = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionBias;
        if (!e_positionBias.isZero())
        {
            lla_positionBias = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionBias) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionBiasUnits::rad_rad_m:
        lla_positionBias = _positionBias;
        break;
    case PositionBiasUnits::deg_deg_m:
        lla_positionBias = Eigen::Vector3d{ deg2rad(_positionBias(0)), deg2rad(_positionBias(1)), _positionBias(2) };
        break;
    }
    LOG_DATA("{}: lla_positionBias = {} [rad, rad, m]", nameId(), lla_positionBias.transpose());

    // Velocity bias in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityBias = Eigen::Vector3d::Zero();
    switch (_velocityBiasUnit)
    {
    case VelocityBiasUnits::m_s:
        n_velocityBias = _velocityBias;
        break;
    }
    LOG_DATA("{}: n_velocityBias = {} [m/s]", nameId(), n_velocityBias.transpose());

    // Roll, pitch, yaw bias in [rad]
    Eigen::Vector3d attitudeBias = Eigen::Vector3d::Zero();
    switch (_attitudeBiasUnit)
    {
    case AttitudeBiasUnits::rad:
        attitudeBias = _attitudeBias;
        break;
    case AttitudeBiasUnits::deg:
        attitudeBias = deg2rad(_attitudeBias);
        break;
    }
    LOG_DATA("{}: attitudeBias = {} [rad]", nameId(), attitudeBias.transpose());

    // #########################################################################################################################################

    // Position Noise standard deviation in latitude, longitude and altitude [rad, rad, m]
    Eigen::Vector3d lla_positionNoiseStd = Eigen::Vector3d::Zero();
    switch (_positionNoiseUnit)
    {
    case PositionNoiseUnits::meter:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise;
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad_rad_m:
        lla_positionNoiseStd = _positionNoise;
        break;
    case PositionNoiseUnits::deg_deg_m:
        lla_positionNoiseStd = deg2rad(_positionNoise);
        break;
    case PositionNoiseUnits::meter2:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise.cwiseSqrt();
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad2_rad2_m2:
        lla_positionNoiseStd = _positionNoise.cwiseSqrt();
        break;
    case PositionNoiseUnits::deg2_deg2_m2:
        lla_positionNoiseStd = deg2rad(_positionNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: lla_positionNoiseStd = {} [rad, rad, m]", nameId(), lla_positionNoiseStd.transpose());

    // Velocity Noise standard deviation in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityNoiseStd = Eigen::Vector3d::Zero();
    switch (_velocityNoiseUnit)
    {
    case VelocityNoiseUnits::m_s:
        n_velocityNoiseStd = _velocityNoise;
        break;
    case VelocityNoiseUnits::m2_s2:
        n_velocityNoiseStd = _velocityNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: n_velocityNoiseStd = {} [m/s]", nameId(), n_velocityNoiseStd.transpose());

    // Attitude Noise standard deviation in [rad]
    Eigen::Vector3d attitudeNoiseStd = Eigen::Vector3d::Zero();
    switch (_attitudeNoiseUnit)
    {
    case AttitudeNoiseUnits::rad:
        attitudeNoiseStd = _attitudeNoise;
        break;
    case AttitudeNoiseUnits::deg:
        attitudeNoiseStd = deg2rad(_attitudeNoise);
        break;
    case AttitudeNoiseUnits::rad2:
        attitudeNoiseStd = _attitudeNoise.cwiseSqrt();
        break;
    case AttitudeNoiseUnits::deg2:
        attitudeNoiseStd = deg2rad(_attitudeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: attitudeNoiseStd = {} [rad]", nameId(), attitudeNoiseStd.transpose());

    // #########################################################################################################################################

    posVelAtt->setState_n(posVelAtt->lla_position()
                              + lla_positionBias
                              + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, lla_positionNoiseStd(0) }(_positionRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, lla_positionNoiseStd(1) }(_positionRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, lla_positionNoiseStd(2) }(_positionRandomNumberGenerator.generator) },
                          posVelAtt->n_velocity()
                              + n_velocityBias
                              + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, n_velocityNoiseStd(0) }(_velocityRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, n_velocityNoiseStd(1) }(_velocityRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, n_velocityNoiseStd(2) }(_velocityRandomNumberGenerator.generator) },
                          trafo::n_Quat_b(posVelAtt->rollPitchYaw()
                                          + attitudeBias
                                          + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, attitudeNoiseStd(0) }(_attitudeRandomNumberGenerator.generator),
                                                             std::normal_distribution<double>{ 0.0, attitudeNoiseStd(1) }(_attitudeRandomNumberGenerator.generator),
                                                             std::normal_distribution<double>{ 0.0, attitudeNoiseStd(2) }(_attitudeRandomNumberGenerator.generator) }));

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, posVelAtt);
}