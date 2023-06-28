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
                                                                   _imuAccelerometerWhiteNoiseUnit == ImuAccelerometerWhiteNoiseUnits::m_s2 ? "Standard deviation"
                                                                                                                                            : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerWhiteNoiseInput.data(), reinterpret_cast<int*>(&_imuAccelerometerWhiteNoiseUnit), "m/s^2\0m^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerWhiteNoiseInput changed to {}", nameId(), _imuAccelerometerWhiteNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerWhiteNoiseUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerWhiteNoiseUnit));
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
                                                                   _imuAccelerometerRandomWalkUnit == ImuAccelerometerRandomWalkUnits::m_s3 ? "Standard deviation"
                                                                                                                                            : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerRandomWalkInput.data(), reinterpret_cast<int*>(&_imuAccelerometerRandomWalkUnit), "m/s^3\0m^2/s^6\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerRandomWalkInput changed to {}", nameId(), _imuAccelerometerRandomWalkInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerRandomWalkUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerRandomWalkUnit));
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
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise Amplitude ##{}", size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuAccelerometerCorrelatedNoiseInput.data(), reinterpret_cast<int*>(&_imuAccelerometerCorrelatedNoiseUnit), "m^2/s^5\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuAccelerometerCorrelatedNoiseInput changed to {}", nameId(), _imuAccelerometerCorrelatedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuAccelerometerCorrelatedNoiseUnit changed to {}", nameId(), fmt::underlying(_imuAccelerometerCorrelatedNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Noise Amplitude of Correlated Noise\n"
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
                                                                   _imuGyroscopeWhiteNoiseUnit == ImuGyroscopeWhiteNoiseUnits::rad_s
                                                                           || _imuGyroscopeWhiteNoiseUnit == ImuGyroscopeWhiteNoiseUnits::deg_s
                                                                       ? "Standard deviation"
                                                                       : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeWhiteNoiseInput.data(), reinterpret_cast<int*>(&_imuGyroscopeWhiteNoiseUnit), "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeWhiteNoiseInput changed to {}", nameId(), _imuGyroscopeWhiteNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeWhiteNoiseUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeWhiteNoiseUnit));
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
                                                                   _imuGyroscopeRandomWalkUnit == ImuGyroscopeRandomWalkUnits::rad_s2
                                                                           || _imuGyroscopeRandomWalkUnit == ImuGyroscopeRandomWalkUnits::deg_s2
                                                                       ? "Standard deviation"
                                                                       : "Variance",
                                                                   size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeRandomWalkInput.data(), reinterpret_cast<int*>(&_imuGyroscopeRandomWalkUnit), "rad/s^2\0deg/s^2\0rad^2/s^4\0deg^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeRandomWalkInput changed to {}", nameId(), _imuGyroscopeRandomWalkInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeRandomWalkUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeRandomWalkUnit));
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
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Correlated Noise Amplitude ##{}", size_t(id))
                                                           .c_str(),
                                                       itemWidth, unitWidth,
                                                       _imuGyroscopeCorrelatedNoiseInput.data(), reinterpret_cast<int*>(&_imuGyroscopeCorrelatedNoiseUnit), "rad^2/s^3\0deg^2/s^3\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: _imuGyroscopeCorrelatedNoiseInput changed to {}", nameId(), _imuGyroscopeCorrelatedNoiseInput.transpose());
                    LOG_DEBUG("{}: _imuGyroscopeCorrelatedNoiseUnit changed to {}", nameId(), fmt::underlying(_imuGyroscopeCorrelatedNoiseUnit));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                    ImGui::TextUnformatted("Noise Amplitude of Correlated Noise\n"
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

    j["imuAccelerometerNoiseUnit"] = _imuAccelerometerWhiteNoiseUnit;
    j["imuAccelerometerWhiteNoise"] = _imuAccelerometerWhiteNoiseInput;
    j["imuAccelerometerRandomWalkUnit"] = _imuAccelerometerRandomWalkUnit;
    j["imuAccelerometerRandomWalk"] = _imuAccelerometerRandomWalkInput;
    j["imuAccelerometerCorrelatedNoiseUnit"] = _imuAccelerometerCorrelatedNoiseUnit;
    j["imuAccelerometerCorrelatedNoise"] = _imuAccelerometerCorrelatedNoiseInput;
    j["imuAccelerometerCorrelatedNoiseCorrelationTime"] = _imuAccelerometerCorrelatedNoiseCorrelationTime;
    j["imuAccelerometerCorrelationTimeUnit"] = _imuAccelerometerCorrelationTimeUnit;
    j["imuAccelerometerRandomNumberGenerator"] = _imuAccelerometerRandomNumberGenerator;
    j["imuGyroscopeNoiseUnit"] = _imuGyroscopeWhiteNoiseUnit;
    j["imuGyroscopeWhiteNoise"] = _imuGyroscopeWhiteNoiseInput;
    j["imuGyroscopeRandomWalkUnit"] = _imuGyroscopeRandomWalkUnit;
    j["imuGyroscopeRandomWalk"] = _imuGyroscopeRandomWalkInput;
    j["imuGyroscopeCorrelatedNoiseUnit"] = _imuGyroscopeCorrelatedNoiseUnit;
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
        j.at("imuAccelerometerNoiseUnit").get_to(_imuAccelerometerWhiteNoiseUnit);
    }
    if (j.contains("imuAccelerometerWhiteNoise"))
    {
        j.at("imuAccelerometerWhiteNoise").get_to(_imuAccelerometerWhiteNoiseInput);
    }
    if (j.contains("imuAccelerometerRandomWalkUnit"))
    {
        j.at("imuAccelerometerRandomWalkUnit").get_to(_imuAccelerometerRandomWalkUnit);
    }
    if (j.contains("imuAccelerometerRandomWalk"))
    {
        j.at("imuAccelerometerRandomWalk").get_to(_imuAccelerometerRandomWalkInput);
    }
    if (j.contains("imuAccelerometerCorrelatedNoiseUnit"))
    {
        j.at("imuAccelerometerCorrelatedNoiseUnit").get_to(_imuAccelerometerCorrelatedNoiseUnit);
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
        j.at("imuGyroscopeNoiseUnit").get_to(_imuGyroscopeWhiteNoiseUnit);
    }
    if (j.contains("imuGyroscopeWhiteNoise"))
    {
        j.at("imuGyroscopeWhiteNoise").get_to(_imuGyroscopeWhiteNoiseInput);
    }
    if (j.contains("imuGyroscopeRandomWalkUnit"))
    {
        j.at("imuGyroscopeRandomWalkUnit").get_to(_imuGyroscopeRandomWalkUnit);
    }
    if (j.contains("imuGyroscopeRandomWalk"))
    {
        j.at("imuGyroscopeRandomWalk").get_to(_imuGyroscopeRandomWalkInput);
    }
    if (j.contains("imuGyroscopeCorrelatedNoiseUnit"))
    {
        j.at("imuGyroscopeCorrelatedNoiseUnit").get_to(_imuGyroscopeCorrelatedNoiseUnit);
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

    _imuAccelerometerRandomWalk = Eigen::Vector3d::Zero();
    _imuGyroscopeRandomWalk = Eigen::Vector3d::Zero();

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
    Eigen::Vector3d accelerometerRandomWalkStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelerometerCorrelatedNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerWhiteNoiseUnit)
    {
    case ImuAccelerometerWhiteNoiseUnits::m_s2:
        accelerometerNoiseStd = _imuAccelerometerWhiteNoiseInput;
        break;
    case ImuAccelerometerWhiteNoiseUnits::m2_s4:
        accelerometerNoiseStd = _imuAccelerometerWhiteNoiseInput.cwiseSqrt();
        break;
    }
    switch (_imuAccelerometerRandomWalkUnit)
    {
    case ImuAccelerometerRandomWalkUnits::m_s3:
        accelerometerRandomWalkStd = _imuAccelerometerRandomWalkInput;
        break;
    case ImuAccelerometerRandomWalkUnits::m2_s6:
        accelerometerRandomWalkStd = _imuAccelerometerRandomWalkInput.cwiseSqrt();
        break;
    }
    accelerometerCorrelatedNoiseStd = ((_dt == 0. ? 0. : 1. / _dt) * _imuAccelerometerCorrelatedNoiseInput).cwiseSqrt();

    LOG_DATA("{}: accelerometerNoiseStd = {} [m/s^2]", nameId(), accelerometerNoiseStd.transpose());
    LOG_DATA("{}: accelerometerRandomWalkStd = {} [m/s^3]", nameId(), accelerometerRandomWalkStd.transpose());
    LOG_DATA("{}: accelerometerCorrelatedNoiseStd = {} [m/s^3]", nameId(), accelerometerCorrelatedNoiseStd.transpose());

    LOG_DATA("{}: accelerometerCorrelatedNoiseCorrelationTime = {} [s]", nameId(), _imuAccelerometerCorrelatedNoiseCorrelationTime.transpose());

    Eigen::Array3d accelerometerCorrelatedNoiseMultiplicationFactor;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        accelerometerCorrelatedNoiseMultiplicationFactor(i) = _imuAccelerometerCorrelatedNoiseCorrelationTime(i) == 0.0 ? 0.0 : static_cast<double>(std::exp(-_dt / std::fabs(_imuAccelerometerCorrelatedNoiseCorrelationTime(i))));
    }

    // Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeNoiseStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeRandomWalkStd = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeCorrelatedNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeWhiteNoiseUnit)
    {
    case ImuGyroscopeWhiteNoiseUnits::rad_s:
        gyroscopeNoiseStd = _imuGyroscopeWhiteNoiseInput;
        break;
    case ImuGyroscopeWhiteNoiseUnits::deg_s:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeWhiteNoiseInput);
        break;
    case ImuGyroscopeWhiteNoiseUnits::rad2_s2:
        gyroscopeNoiseStd = _imuGyroscopeWhiteNoiseInput.cwiseSqrt();
        break;
    case ImuGyroscopeWhiteNoiseUnits::deg2_s2:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeWhiteNoiseInput.cwiseSqrt());
        break;
    }
    switch (_imuGyroscopeRandomWalkUnit)
    {
    case ImuGyroscopeRandomWalkUnits::rad_s2:
        gyroscopeRandomWalkStd = _imuGyroscopeRandomWalkInput;
        break;
    case ImuGyroscopeRandomWalkUnits::deg_s2:
        gyroscopeRandomWalkStd = deg2rad(_imuGyroscopeRandomWalkInput);
        break;
    case ImuGyroscopeRandomWalkUnits::rad2_s4:
        gyroscopeRandomWalkStd = _imuGyroscopeRandomWalkInput.cwiseSqrt();
        break;
    case ImuGyroscopeRandomWalkUnits::deg2_s4:
        gyroscopeRandomWalkStd = deg2rad(_imuGyroscopeRandomWalkInput.cwiseSqrt());
        break;
    }
    switch (_imuGyroscopeCorrelatedNoiseUnit)
    {
    case ImuGyroscopeCorrelatedNoiseUnits::rad2_s3:
        gyroscopeCorrelatedNoiseStd = ((_dt == 0. ? 0. : 1. / _dt) * _imuGyroscopeCorrelatedNoiseInput).cwiseSqrt();
        break;
    case ImuGyroscopeCorrelatedNoiseUnits::deg2_s3:
        gyroscopeCorrelatedNoiseStd = ((_dt == 0. ? 0. : 1. / _dt) * (_imuGyroscopeCorrelatedNoiseInput * pow(std::numbers::pi_v<double> / 180.0, 2))).cwiseSqrt();
        break;
    }

    LOG_DATA("{}: gyroscopeNoiseStd = {} [rad/s]", nameId(), gyroscopeNoiseStd.transpose());
    LOG_DATA("{}: gyroscopeRandomWalkStd = {} [rad/s]", nameId(), gyroscopeRandomWalkStd.transpose());
    LOG_DATA("{}: gyroscopeCorrelatedNoiseStd = {} [rad/s]", nameId(), gyroscopeCorrelatedNoiseStd.transpose());

    LOG_DATA("{}: gyroscopeCorrelatedNoiseCorrelationTIme = {} [rad/s]", nameId(), _imuGyroscopeCorrelatedNoiseCorrelationTime.transpose());

    Eigen::Array3d gyroscopeCorrelatedNoiseMultiplicationFactor;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        gyroscopeCorrelatedNoiseMultiplicationFactor(i) = _imuGyroscopeCorrelatedNoiseCorrelationTime(i) == 0.0 ? 0.0 : static_cast<double>(std::exp(-_dt / std::fabs(_imuGyroscopeCorrelatedNoiseCorrelationTime(i))));
    }

    // #########################################################################################################################################

    _imuAccelerometerRandomWalk += _dt
                                   * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerRandomWalkStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                      std::normal_distribution<double>{ 0.0, accelerometerRandomWalkStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                      std::normal_distribution<double>{ 0.0, accelerometerRandomWalkStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) };

    _imuGyroscopeRandomWalk += _dt
                               * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeRandomWalkStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                  std::normal_distribution<double>{ 0.0, gyroscopeRandomWalkStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                  std::normal_distribution<double>{ 0.0, gyroscopeRandomWalkStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) };

    _imuAccelerometerCorrelatedNoise = (_imuAccelerometerCorrelatedNoise.array() * accelerometerCorrelatedNoiseMultiplicationFactor).matrix()
                                       + _dt
                                             * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                                std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                                std::normal_distribution<double>{ 0.0, accelerometerCorrelatedNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) };

    _imuGyroscopeCorrelatedNoise = (_imuGyroscopeCorrelatedNoise.array() * gyroscopeCorrelatedNoiseMultiplicationFactor).matrix()
                                   + _dt
                                         * Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, gyroscopeCorrelatedNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) };

    imuObs->accelUncompXYZ.value() += accelerometerBias_p
                                      + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) }
                                      + _imuAccelerometerRandomWalk
                                      + _imuAccelerometerCorrelatedNoise;

    imuObs->gyroUncompXYZ.value() += gyroscopeBias_p
                                     + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) }
                                     + _imuGyroscopeRandomWalk
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