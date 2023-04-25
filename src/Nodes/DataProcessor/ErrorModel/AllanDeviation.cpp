// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AllanDeviation.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <chrono>

NAV::AllanDeviation::AllanDeviation()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    // _lockConfigDuringRun = false;
    _guiConfigDefaultWindowSize = { 630, 410 };

    nm::CreateOutputPin(this, "Object", Pin::Type::Object, { "AdevOutput" }, &_valueObject);

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &AllanDeviation::receiveImuObs);
}

NAV::AllanDeviation::~AllanDeviation()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::AllanDeviation::typeStatic()
{
    return "AllanDeviation";
}

std::string NAV::AllanDeviation::type() const
{
    return typeStatic();
}

std::string NAV::AllanDeviation::category()
{
    return "Data Processor";
}

void NAV::AllanDeviation::guiConfig()
{
    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("AllanDeviationTabBar", tab_bar_flags))
    {
        if (ImGui::BeginTabItem("Accelerometer"))
        {
            if (ImPlot::BeginPlot("Allan Deviation of Accelerometer"))
            {
                ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                ImPlot::SetupAxes("τ [s]", "σ [m/s²]", ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("x", _averagingTimes.data(), _accelAllanDeviation.at(0).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::PlotLine("y", _averagingTimes.data(), _accelAllanDeviation.at(1).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::PlotLine("z", _averagingTimes.data(), _accelAllanDeviation.at(2).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Gyroscope"))
        {
            if (ImPlot::BeginPlot("Allan Deviation of Gyroscope"))
            {
                ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                ImPlot::SetupAxes("τ [s]", "σ [rad/s]", ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("x", _averagingTimes.data(), _gyroAllanDeviation.at(0).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::PlotLine("y", _averagingTimes.data(), _gyroAllanDeviation.at(1).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::PlotLine("z", _averagingTimes.data(), _gyroAllanDeviation.at(2).data(), static_cast<int>(_averagingTimes.size()));
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

[[nodiscard]] json NAV::AllanDeviation::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // j["outputFrequency"] = _outputFrequency;

    return j;
}

void NAV::AllanDeviation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency"))
    {
        // j.at("outputFrequency").get_to(_outputFrequency);
    }
}

bool NAV::AllanDeviation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startingInsTime.reset();

    _accelCumSum = std::vector<Eigen::Vector3d>{ Eigen::Vector3d::Zero() };
    _gyroCumSum = std::vector<Eigen::Vector3d>{ Eigen::Vector3d::Zero() };

    _averagingFactors = std::vector<double>{};
    _averagingTimes = std::vector<double>{};
    _observationCount = std::vector<double>{};

    _accelAllanSum = std::array<std::vector<double>, 3>{};
    _gyroAllanSum = std::array<std::vector<double>, 3>{};

    _accelAllanVariance = std::array<std::vector<double>, 3>{};
    _gyroAllanVariance = std::array<std::vector<double>, 3>{};

    _accelAllanDeviation = std::array<std::vector<double>, 3>{};
    _gyroAllanDeviation = std::array<std::vector<double>, 3>{};

    _cumSumLength = 1;

    _nextAveragingFactorExponent = 1;

    _nextAveragingFactor = 1;

    return true;
}

void NAV::AllanDeviation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::AllanDeviation::receiveImuObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    // save InsTime of first imuObs for sampling interval computation
    if (_startingInsTime.empty())
    {
        _startingInsTime = obs->insTime;
    }

    // cumulative sums
    _accelCumSum.push_back(_accelCumSum.back() + obs->accelUncompXYZ.value());
    _gyroCumSum.push_back(_gyroCumSum.back() + obs->gyroUncompXYZ.value());
    _cumSumLength = static_cast<unsigned int>(_accelCumSum.size());

    // extending _averagingFactors if necessary
    if (_cumSumLength - 1 == _nextAveragingFactor * 2)
    {
        _averagingFactors.push_back(_nextAveragingFactor);
        _observationCount.push_back(0);
        for (size_t i = 0; i < 3; i++)
        {
            _accelAllanSum.at(i).push_back(0);
            _gyroAllanSum.at(i).push_back(0);
        }

        // computation of next averaging factor
        while (static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade))) == _nextAveragingFactor)
        {
            _nextAveragingFactorExponent++;
        }

        _nextAveragingFactor = static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade)));
    }

    // computation Allan sum
    for (size_t i = 0; i < _averagingFactors.size(); i++)
    {
        _accelTempSum = _accelCumSum.at(_cumSumLength - 1)
                        - 2 * _accelCumSum.at(_cumSumLength - 1 - static_cast<unsigned int>(_averagingFactors.at(i)))
                        + _accelCumSum.at(_cumSumLength - 1 - 2 * static_cast<unsigned int>(_averagingFactors.at(i)));
        _gyroTempSum = _gyroCumSum.at(_cumSumLength - 1)
                       - 2 * _gyroCumSum.at(_cumSumLength - 1 - static_cast<unsigned int>(_averagingFactors.at(i)))
                       + _gyroCumSum.at(_cumSumLength - 1 - 2 * static_cast<unsigned int>(_averagingFactors.at(i)));

        for (size_t j = 0; j < 3; j++)
        {
            _accelAllanSum.at(j).at(i) += pow(_accelTempSum(static_cast<long>(j)), 2);
            _gyroAllanSum.at(j).at(i) += pow(_gyroTempSum(static_cast<long>(j)), 2);
        }
        _observationCount.at(i)++;
    }

    // computation of Allan Variance and Deviation
    if (_cumSumLength % 1 == 0)
    {
        _samplingInterval = static_cast<double>((obs->insTime - _startingInsTime).count()) / (_cumSumLength - 1.);

        _averagingTimes.resize(_averagingFactors.size(), 0.);
        for (size_t i = 0; i < _averagingFactors.size(); i++)
        {
            _averagingTimes.at(i) = _averagingFactors.at(i) * _samplingInterval;
        }

        for (size_t j = 0; j < 3; j++)
        {
            _accelAllanVariance.at(j).resize(_averagingFactors.size(), 0.);
            _gyroAllanVariance.at(j).resize(_averagingFactors.size(), 0.);
            _accelAllanDeviation.at(j).resize(_averagingFactors.size(), 0.);
            _gyroAllanDeviation.at(j).resize(_averagingFactors.size(), 0.);

            for (size_t i = 0; i < _averagingFactors.size(); i++)
            {
                _accelAllanVariance.at(j).at(i) = _accelAllanSum.at(j).at(i) / (2 * pow(_averagingFactors.at(i), 2) * _observationCount.at(i));
                _gyroAllanVariance.at(j).at(i) = _gyroAllanSum.at(j).at(i) / (2 * pow(_averagingFactors.at(i), 2) * _observationCount.at(i));

                _accelAllanDeviation.at(j).at(i) = sqrt(_accelAllanVariance.at(j).at(i));
                _gyroAllanDeviation.at(j).at(i) = sqrt(_gyroAllanVariance.at(j).at(i));
            }
        }
    }

    notifyOutputValueChanged(OUTPUT_PORT_INDEX_ADEV_OUTPUT, obs->insTime); // TODO: lock thread when modifying output value
}
