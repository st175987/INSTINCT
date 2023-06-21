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

#include <unsupported/Eigen/MatrixFunctions>

NAV::AllanDeviation::AllanDeviation()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _lockConfigDuringRun = false;
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
    const std::array<char[2], 3> legendEntries{ "x", "y", "z" };

    static double slopeTicks[] = { -2, -1, 0, 1, 2 };

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("AllanDeviationTabBar", tab_bar_flags))
    {
        if (ImGui::BeginTabItem("Accelerometer"))
        {
            if (ImPlot::BeginPlot("Allan Deviation of Accelerometer"))
            {
                ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                ImPlot::SetupAxes("τ [s]", "σ [m/s²]", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
                ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, _confidenceFillAlpha);
                for (size_t d = 0; d < 3; d++)
                {
                    if (_displayConfidence & !_averagingTimes.empty())
                    {
                        ImPlot::PlotShaded(legendEntries.at(d), _averagingTimes.data(), _allanDeviationConfidenceIntervals.at(0).at(d).at(0).data(), _allanDeviationConfidenceIntervals.at(0).at(d).at(1).data(), static_cast<int>(_averagingTimes.size()));
                    }
                    ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _allanDeviation.at(0).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    if (_displayEstimation & !_averagingTimes.empty())
                    {
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _estimatedAllanDeviation.at(0).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    }
                }
                ImPlot::EndPlot();
            }
            if (ImGui::TreeNode("Slopes"))
            {
                if (ImPlot::BeginPlot("Slopes of Allan Variance"))
                {
                    ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                    ImPlot::SetupAxis(ImAxis_X1, "τ [s]", ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupAxis(ImAxis_Y1, "µ [ ]", ImPlotAxisFlags_Lock);
                    ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5);
                    ImPlot::SetupAxisTicks(ImAxis_Y1, slopeTicks, 5, NULL, false);
                    for (size_t d = 0; d < 3; d++)
                    {
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _slope.at(0).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    }
                    ImPlot::EndPlot();
                }

                ImGui::TreePop();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Gyroscope"))
        {
            if (ImPlot::BeginPlot("Allan Deviation of Gyroscope"))
            {
                ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, _confidenceFillAlpha);
                ImPlot::SetupAxes("τ [s]", "σ [rad/s]", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
                for (size_t d = 0; d < 3; d++)
                {
                    if (_displayConfidence & !_averagingTimes.empty())
                    {
                        ImPlot::PlotShaded(legendEntries.at(d), _averagingTimes.data(), _allanDeviationConfidenceIntervals.at(1).at(d).at(0).data(), _allanDeviationConfidenceIntervals.at(1).at(d).at(1).data(), static_cast<int>(_averagingTimes.size()));
                    }
                    ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _allanDeviation.at(1).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    if (_displayEstimation & !_averagingTimes.empty())
                    {
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _estimatedAllanDeviation.at(1).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    }
                }
                ImPlot::EndPlot();
            }
            if (ImGui::TreeNode("Slopes"))
            {
                if (ImPlot::BeginPlot("Slopes of Allan Variance"))
                {
                    ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                    ImPlot::SetupAxis(ImAxis_X1, "τ [s]", ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupAxis(ImAxis_Y1, "µ [ ]", ImPlotAxisFlags_Lock);
                    ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5);
                    ImPlot::SetupAxisTicks(ImAxis_Y1, slopeTicks, 5, NULL, false);
                    for (size_t d = 0; d < 3; d++)
                    {
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _slope.at(1).at(d).data(), static_cast<int>(_averagingTimes.size()));
                    }
                    ImPlot::EndPlot();
                }

                ImGui::TreePop();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        ImGui::Checkbox("Display Confidence Intervals", &_displayConfidence);
        if (!_displayConfidence)
            ImGui::BeginDisabled();
        ImGui::SliderFloat("Confidence Alpha Channel", &_confidenceFillAlpha, 0.0f, 1.0f, "%.2f");
        if (!_displayConfidence)
            ImGui::EndDisabled();
        ImGui::Checkbox("Compute Allan Deviation last", &_updateLast);
        ImGui::Checkbox("Display Estimation", &_displayEstimation);
    }
}

[[nodiscard]] json NAV::AllanDeviation::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["displayConfidence"] = _displayConfidence;
    j["confidenceFillAlpha"] = _confidenceFillAlpha;
    j["updateLast"] = _updateLast;
    j["displayEstimation"] = _displayEstimation;

    return j;
}

void NAV::AllanDeviation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("displayConfidence"))
    {
        j.at("displayConfidence").get_to(_displayConfidence);
    }
    if (j.contains("confidenceFillAlpha"))
    {
        j.at("confidenceFillAlpha").get_to(_confidenceFillAlpha);
    }
    if (j.contains("updateLast"))
    {
        j.at("updateLast").get_to(_updateLast);
    }
    if (j.contains("displayEstimation"))
    {
        j.at("displayEstimation").get_to(_displayEstimation);
    }
}

bool NAV::AllanDeviation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startingInsTime.reset();

    _cumSum = std::array<std::vector<Eigen::Vector3d>, 2>{ { { Eigen::Vector3d::Zero() }, { Eigen::Vector3d::Zero() } } };

    _averagingFactors = std::vector<double>{};
    _averagingTimes = std::vector<double>{};
    _observationCount = std::vector<double>{};

    _allanSum = std::array<std::array<std::vector<double>, 3>, 2>{};

    _allanVariance = std::array<std::array<std::vector<double>, 3>, 2>{};

    _allanDeviation = std::array<std::array<std::vector<double>, 3>, 2>{};

    _slope = std::array<std::array<std::vector<double>, 3>, 2>{};

    _allanDeviationConfidenceIntervals = std::array<std::array<std::array<std::vector<double>, 2>, 3>, 2>{};

    _imuObsCount = 0;

    _averagingFactorCount = 0;

    _nextAveragingFactorExponent = 1;

    _nextAveragingFactor = 1;

    _S_N = std::array<std::array<double, 3>, 2>{};

    _estimatedAllanDeviation = std::array<std::array<std::vector<double>, 3>, 2>{};

    return true;
}

void NAV::AllanDeviation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::AllanDeviation::receiveImuObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    bool lastMessage = false;
    if (queue.empty()
        && inputPins[INPUT_PORT_INDEX_IMU_OBS].isPinLinked()
        && inputPins[INPUT_PORT_INDEX_IMU_OBS].link.getConnectedPin()->mode == OutputPin::Mode::REAL_TIME)
    {
        lastMessage = true;
    }

    // save InsTime of first imuObs for sampling interval computation
    if (_startingInsTime.empty())
    {
        _startingInsTime = obs->insTime;
    }

    _imuObsCount++;
    _cumSum.at(0).push_back(_cumSum.at(0).back() + obs->accelUncompXYZ.value());
    _cumSum.at(1).push_back(_cumSum.at(1).back() + obs->gyroUncompXYZ.value());

    // extending _averagingFactors if necessary
    if (_imuObsCount == _nextAveragingFactor * 2)
    {
        _averagingFactors.push_back(_nextAveragingFactor);
        _averagingFactorCount += 1;
        _observationCount.push_back(0);

        for (size_t s = 0; s < 2; s++)
        {
            for (size_t d = 0; d < 3; d++)
            {
                _allanSum.at(s).at(d).push_back(0);
            }
        }

        // computation of next averaging factor
        while (static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade))) == _nextAveragingFactor)
        {
            _nextAveragingFactorExponent++;
        }

        _nextAveragingFactor = static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade)));
    }

    // computation Allan sum
    for (size_t k = 0; k < _averagingFactorCount; k++)
    {
        for (size_t s = 0; s < 2; s++)
        {
            Eigen::Vector3d tempSum = _cumSum.at(s).at(_imuObsCount)
                                      - 2 * _cumSum.at(s).at(_imuObsCount - static_cast<unsigned int>(_averagingFactors.at(k)))
                                      + _cumSum.at(s).at(_imuObsCount - 2 * static_cast<unsigned int>(_averagingFactors.at(k)));

            for (size_t d = 0; d < 3; d++)
            {
                _allanSum.at(s).at(d).at(k) += pow(tempSum(static_cast<long>(d)), 2);
            }
        }
        _observationCount.at(k)++;
    }

    // computation of Allan Variance and Deviation
    if (_updateLast ? lastMessage : _imuObsCount % 1 == 0)
    {
        _samplingInterval = static_cast<double>((obs->insTime - _startingInsTime).count()) / _imuObsCount;
        _averagingTimes.resize(_averagingFactorCount, 0.);
        _confidenceMultiplicationFactor.resize(_averagingFactorCount, 0.);
        for (size_t k = 0; k < _averagingFactorCount; k++)
        {
            _averagingTimes.at(k) = _averagingFactors.at(k) * _samplingInterval;
            _confidenceMultiplicationFactor.at(k) = sqrt(0.5 / (_imuObsCount / _averagingFactors.at(k) - 1));
        }

        for (size_t s = 0; s < 2; s++)
        {
            for (size_t d = 0; d < 3; d++)
            {
                _allanVariance.at(s).at(d).resize(_averagingFactorCount, 0.);
                _allanDeviation.at(s).at(d).resize(_averagingFactorCount, 0.);

                for (size_t j = 0; j < 2; j++)
                {
                    _allanDeviationConfidenceIntervals.at(s).at(d).at(j).resize(_averagingFactorCount, 0.);
                }

                for (size_t k = 0; k < _averagingFactorCount; k++)
                {
                    _allanVariance.at(s).at(d).at(k) = _allanSum.at(s).at(d).at(k) / (2 * pow(_averagingFactors.at(k), 2) * _observationCount.at(k));

                    _allanDeviation.at(s).at(d).at(k) = sqrt(_allanVariance.at(s).at(d).at(k));

                    _allanDeviationConfidenceIntervals.at(s).at(d).at(0).at(k) = _allanDeviation.at(s).at(d).at(k) * (1 - _confidenceMultiplicationFactor.at(k));
                    _allanDeviationConfidenceIntervals.at(s).at(d).at(1).at(k) = _allanDeviation.at(s).at(d).at(k) * (1 + _confidenceMultiplicationFactor.at(k));
                }
            }
        }

        computeSlopes();
        estimateNoiseParameters();
    }

    notifyOutputValueChanged(OUTPUT_PORT_INDEX_ADEV_OUTPUT, obs->insTime); // TODO: lock thread when modifying output value
}

void NAV::AllanDeviation::computeSlopes()
{
    unsigned long lo, hi;
    double divisor;

    for (size_t s = 0; s < 2; s++)
    {
        for (size_t d = 0; d < 3; d++)
        {
            _slope.at(s).at(d).resize(_averagingFactorCount, 0.);
        }

        for (size_t k = 0; k < _averagingFactorCount; k++)
        {
            lo = (k == 0 ? k : k - 1);
            hi = (k == _averagingFactorCount - 1 ? k : k + 1);

            divisor = log(_averagingFactors.at(hi) / _averagingFactors.at(lo));

            for (size_t d = 0; d < 3; d++)
            {
                _slope.at(s).at(d).at(k) = log(_allanVariance.at(s).at(d).at(hi) / _allanVariance.at(s).at(d).at(lo)) / divisor;
            }
        }
    }
}

void NAV::AllanDeviation::estimateNoiseParameters()
{
    for (size_t s = 0; s < 2; s++)
    {
        for (size_t d = 0; d < 3; d++)
        {
            std::vector<double> avarN;
            std::vector<double> weightsN;
            std::vector<double> averagingTimesInv;

            for (size_t k = 0; k < _averagingFactorCount; k++)
            {
                if (_slope.at(s).at(d).at(k) > -1.5 && _slope.at(s).at(d).at(k) < -0.5)
                {
                    avarN.push_back(_allanVariance.at(s).at(d).at(k));
                    weightsN.push_back(1 / pow(2. * _confidenceMultiplicationFactor.at(k) * _allanVariance.at(s).at(d).at(k), 2.));
                    averagingTimesInv.push_back(1 / _averagingTimes.at(k));
                }
            }

            long m = static_cast<long>(avarN.size());

            Eigen::Map<Eigen::VectorXd> y(avarN.data(), m);
            Eigen::Map<Eigen::VectorXd> A(averagingTimesInv.data(), m);
            Eigen::Map<Eigen::VectorXd> p(weightsN.data(), m);
            Eigen::VectorXd x;

            if (m > 0)
            {
                x = (A.transpose() * p.asDiagonal() * A).inverse() * A.transpose() * p.asDiagonal() * y;
                _S_N.at(s).at(d) = x(0);
            }
            else
            {
                _S_N.at(s).at(d) = 0;
            }

            _estimatedAllanDeviation.at(s).at(d).resize(_averagingFactorCount, 0.);

            double N = sqrt(_S_N.at(s).at(d));

            for (size_t k = 0; k < _averagingFactorCount; k++)
            {
                _estimatedAllanDeviation.at(s).at(d).at(k) = N / sqrt(_averagingTimes.at(k));
            }
        }
    }
}