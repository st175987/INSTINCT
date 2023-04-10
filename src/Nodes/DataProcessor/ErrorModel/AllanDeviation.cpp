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
    if (ImPlot::BeginPlot("Line Plot"))
    {
        ImPlot::SetupAxes("tau", "sigma", ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_LogScale + ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("avar x", _averagingFactors.data(), _accelAllanVariance.at(0).data(), static_cast<int>(_averagingFactors.size()));
        ImPlot::PlotLine("avar y", _averagingFactors.data(), _accelAllanVariance.at(1).data(), static_cast<int>(_averagingFactors.size()));
        ImPlot::PlotLine("avar z", _averagingFactors.data(), _accelAllanVariance.at(2).data(), static_cast<int>(_averagingFactors.size()));
        ImPlot::EndPlot();
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

    return true;
}

void NAV::AllanDeviation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::AllanDeviation::receiveImuObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    _accelCumSum.push_back(_accelCumSum.back() + obs->accelUncompXYZ.value());
    _gyroCumSum.push_back(_accelCumSum.back() + obs->gyroUncompXYZ.value());
    _cumSumLength = static_cast<unsigned int>(_accelCumSum.size());

    if (_cumSumLength - 1 == _nextAveragingFactor * 2)
    {
        _averagingFactors.push_back(_nextAveragingFactor);
        _observationCount.push_back(0);
        for (size_t i = 0; i < 3; i++)
        {
            _accelAllanSum.at(i).push_back(0);
            _gyroAllanSum.at(i).push_back(0);
            _accelAllanVariance.at(i).push_back(0);
            _gyroAllanVariance.at(i).push_back(0);
        }

        while (static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade))) == _nextAveragingFactor)
        {
            _nextAveragingFactorExponent++;
        }

        _nextAveragingFactor = static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade)));
    }

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

    if (_cumSumLength % 1000 == 0)
    {
        for (size_t i = 0; i < _averagingFactors.size(); i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                _accelAllanVariance.at(j).at(i) = _accelAllanSum.at(j).at(i) / (2 * pow(_averagingFactors.at(i), 2) * _observationCount.at(i));
                _gyroAllanVariance.at(j).at(i) = _gyroAllanSum.at(j).at(i) / (2 * pow(_averagingFactors.at(i), 2) * _observationCount.at(i));
            }
        }
    }

    notifyOutputValueChanged(OUTPUT_PORT_INDEX_ADEV_OUTPUT, obs->insTime); // TODO: lock thread when modifying output value
}
