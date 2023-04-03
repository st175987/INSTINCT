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
    _vectorLength = static_cast<unsigned int>(_accelCumSum.size());

    if (_vectorLength - 1 == _nextAveragingFactor * 2)
    {
        _averagingFactors.push_back(_nextAveragingFactor);
        _observationCount.push_back(0);
        _accelAllanSum.push_back(Eigen::Vector3d::Zero());
        _gyroAllanSum.push_back(Eigen::Vector3d::Zero());
        _accelAllanVariance.push_back(Eigen::Vector3d::Zero());
        _gyroAllanVariance.push_back(Eigen::Vector3d::Zero());
        while (static_cast<unsigned int>(round(pow(10, _nextAveragingFactorExponent / _averagingFactorsPerDecade))) == _nextAveragingFactor)
        {
            _nextAveragingFactorExponent++;
        }
        _nextAveragingFactor = static_cast<unsigned int>(round(pow(10, _nextAveragingFactorExponent / _averagingFactorsPerDecade)));
    }

    for (size_t i = 0; i < _averagingFactors.size(); i++)
    {
        _accelTempSum = _accelCumSum[_vectorLength + 1] - 2 * _accelCumSum[_vectorLength + 1 - _averagingFactors[i]] + _accelCumSum[_vectorLength + 1 - 2 * _averagingFactors[i]];
        _accelAllanSum[i] += _accelTempSum.cwiseProduct(_accelTempSum);
        _gyroTempSum = _gyroCumSum[_vectorLength + 1] - 2 * _gyroCumSum[_vectorLength + 1 - _averagingFactors[i]] + _gyroCumSum[_vectorLength + 1 - 2 * _averagingFactors[i]];
        _gyroAllanSum[i] += _gyroTempSum.cwiseProduct(_gyroTempSum);
        _observationCount[i]++;
    }

    if (_vectorLength % 1000 == 0)
    {
        for (size_t i = 0; i < _averagingFactors.size(); i++)
        {
            _accelAllanVariance[i] = _accelAllanSum[i] / (pow(_averagingFactors[i], 2) * _observationCount[i]);
            _gyroAllanVariance[i] = _gyroAllanSum[i] / (pow(_averagingFactors[i], 2) * _observationCount[i]);
        }
    }

    notifyOutputValueChanged(OUTPUT_PORT_INDEX_ADEV_OUTPUT, obs->insTime); // TODO: lock thread when modifying output value
}
