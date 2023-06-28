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
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _estimatedAllanDeviation.at(0).at(d).data(), static_cast<int>(_averagingTimes.size()), ImPlotLineFlags_Segments);
                    }
                    if (_displayBiasInstability)
                    {
                        ImPlot::SetNextMarkerStyle(7, 6, IMPLOT_AUTO_COL, 0.5);
                        double sigma[1] = { _biasInstability.at(0).at(d) };
                        double tau[1] = { _biasInstabilityTau.at(0).at(d) };
                        ImPlot::PlotLine(legendEntries.at(d), tau, sigma, 1);
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
                        ImPlot::PlotLine(legendEntries.at(d), _averagingTimes.data(), _estimatedAllanDeviation.at(1).at(d).data(), static_cast<int>(_averagingTimes.size()), ImPlotLineFlags_Segments);
                    }
                    if (_displayBiasInstability)
                    {
                        ImPlot::SetNextMarkerStyle(7, 6, IMPLOT_AUTO_COL, 0.5);
                        double sigma[1] = { _biasInstability.at(1).at(d) };
                        double tau[1] = { _biasInstabilityTau.at(1).at(d) };
                        ImPlot::PlotLine(legendEntries.at(d), tau, sigma, 1);
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
        ImGui::SameLine(200);
        ImGui::Checkbox("Display Bias Instability", &_displayBiasInstability);
        if (ImGui::TreeNode("Estimation Parameters"))
        {
            ImGui::Checkbox("Random Walk", &_estimateRandomWalk);
            ImGui::SameLine(200);
            ImGui::Checkbox("Correlated Noise", &_estimateCorrelatedNoise);
            ImGui::TreePop();
        }
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
    j["displayBiasInstability"] = _displayBiasInstability;
    j["estimateRandomWalk"] = _estimateRandomWalk;
    j["estimateCorrelatedNoise"] = _estimateCorrelatedNoise;

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
    if (j.contains("displayBiasInstability"))
    {
        j.at("displayBiasInstability").get_to(_displayBiasInstability);
    }
    if (j.contains("estimateRandomWalk"))
    {
        j.at("estimateRandomWalk").get_to(_estimateRandomWalk);
    }
    if (j.contains("estimateCorrelatedNoise"))
    {
        j.at("estimateCorrelatedNoise").get_to(_estimateCorrelatedNoise);
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
    _S_K = std::array<std::array<double, 3>, 2>{};
    _S_G = std::array<std::array<double, 3>, 2>{};
    _tau_G = std::array<std::array<double, 3>, 2>{};
    _biasInstability = std::array<std::array<double, 3>, 2>{};
    _biasInstabilityTau = std::array<std::array<double, 3>, 2>{};

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
            // white noise estimation
            std::vector<double> avar_N;    // observations
            std::vector<double> weights_N; // weights
            std::vector<double> vec_A_N;   // design matrix

            // only use data where slope is between -1.5 and -0.5
            for (size_t k = 0; k < _averagingFactorCount; k++)
            {
                if (_slope.at(s).at(d).at(k) > -1.5 && _slope.at(s).at(d).at(k) < -0.5)
                {
                    avar_N.push_back(_allanVariance.at(s).at(d).at(k));
                    weights_N.push_back(1 / pow(2. * _confidenceMultiplicationFactor.at(k) * _allanVariance.at(s).at(d).at(k), 2.));
                    vec_A_N.push_back(1 / _averagingTimes.at(k));
                }
            }

            long m_N = static_cast<long>(avar_N.size());

            // least squares
            if (m_N > 0)
            {
                // convert to Eigen data types
                Eigen::Map<Eigen::VectorXd> y_N(avar_N.data(), m_N);
                Eigen::Map<Eigen::VectorXd> A_N(vec_A_N.data(), m_N);
                Eigen::Map<Eigen::VectorXd> p_N(weights_N.data(), m_N);
                Eigen::VectorXd x_N;

                x_N = (A_N.transpose() * p_N.asDiagonal() * A_N).inverse() * A_N.transpose() * p_N.asDiagonal() * y_N;
                _S_N.at(s).at(d) = x_N(0);
            }
            else
            {
                _S_N.at(s).at(d) = 0;
            }

            // bias instability computation

            if (_averagingFactorCount > _averagingFactorsPerDecade)
            {
                auto min_avar = std::min_element(_allanVariance.at(s).at(d).begin(),
                                                 _allanVariance.at(s).at(d).end() - static_cast<long>(_averagingFactorsPerDecade));
                unsigned long idx_min = static_cast<unsigned long>(std::distance(std::begin(_allanVariance.at(s).at(d)), min_avar));

                if (_slope.at(s).at(d).at(idx_min) < 0.1 && _slope.at(s).at(d).at(idx_min) > -0.1)
                {
                    _biasInstability.at(s).at(d) = _allanDeviation.at(s).at(d).at(idx_min);
                    _biasInstabilityTau.at(s).at(d) = _averagingTimes.at(idx_min);
                }
                else
                {
                    _biasInstability.at(s).at(d) = 0.;
                    _biasInstabilityTau.at(s).at(d) = 0.;
                }
            }

            // random walk estimation
            if (_estimateRandomWalk)
            {
                std::vector<double> avar_K;    // observations
                std::vector<double> weights_K; // weights
                std::vector<double> vec_A_K;   // design matrix

                // only use data where slope is between 0.5 and 1.5 and is above white noise
                for (size_t k = 0; k < _averagingFactorCount; k++)
                {
                    double tempAvar = _allanVariance.at(s).at(d).at(k) - _S_N.at(s).at(d) / _averagingTimes.at(k);
                    if (_slope.at(s).at(d).at(k) > 0.5 && _slope.at(s).at(d).at(k) < 1.5 && tempAvar > 0.)
                    {
                        avar_K.push_back(tempAvar);
                        weights_K.push_back(1 / pow(2. * _confidenceMultiplicationFactor.at(k) * _allanVariance.at(s).at(d).at(k), 2.));
                        vec_A_K.push_back(_averagingTimes.at(k) / 3);
                    }
                }

                long m_K = static_cast<long>(avar_K.size());

                // least squares
                if (m_K > 0)
                {
                    // convert to Eigen data types
                    Eigen::Map<Eigen::VectorXd> y_K(avar_K.data(), m_K);
                    Eigen::Map<Eigen::VectorXd> A_K(vec_A_K.data(), m_K);
                    Eigen::Map<Eigen::VectorXd> p_K(weights_K.data(), m_K);
                    Eigen::VectorXd x_K;

                    x_K = (A_K.transpose() * p_K.asDiagonal() * A_K).inverse() * A_K.transpose() * p_K.asDiagonal() * y_K;
                    _S_K.at(s).at(d) = x_K(0);
                }
                else
                {
                    _S_K.at(s).at(d) = 0;
                }
            }
            else
            {
                _S_K.at(s).at(d) = 0;
            }

            // correlated noise estimation
            if (_estimateCorrelatedNoise)
            {
                std::vector<double> avar_G;    // observations
                std::vector<double> weights_G; // weights
                std::vector<double> taus_G;    // averaging times

                // only use data where slope is between -0.5 and 0.5 and is above white noise and random walk noise
                for (size_t k = 0; k < _averagingFactorCount; k++)
                {
                    double tempAvar = _allanVariance.at(s).at(d).at(k)
                                      - _S_N.at(s).at(d) / _averagingTimes.at(k)
                                      - _S_K.at(s).at(d) / 3 * _averagingTimes.at(k);
                    if (_slope.at(s).at(d).at(k) > -0.5 && _slope.at(s).at(d).at(k) < 0.5 && tempAvar > 0.)
                    {
                        avar_G.push_back(tempAvar);
                        weights_G.push_back(1 / pow(2. * _confidenceMultiplicationFactor.at(k) * _allanVariance.at(s).at(d).at(k), 2.));
                        taus_G.push_back(_averagingTimes.at(k));
                    }
                }

                long m_G = static_cast<long>(avar_G.size());

                if (m_G > 0)
                {
                    Eigen::Vector2d x_G_0;

                    if (_estimateRandomWalk && _S_K.at(s).at(d) != 0) // compute S_G and tau_G with random walk noise
                    {
                        double tau_min = sqrt(3 * _S_N.at(s).at(d) / _S_K.at(s).at(d));
                        x_G_0(1) = tau_min / _gamma_tau;

                        double sigma_min = 2 * sqrt(_S_N.at(s).at(d) * _S_K.at(s).at(d) / 3);

                        auto upper_tau = std::upper_bound(_averagingTimes.begin(), _averagingTimes.end(), tau_min);
                        unsigned long upper_tau_idx = static_cast<unsigned long>(upper_tau - _averagingTimes.begin());
                        unsigned long idx = (upper_tau_idx == _averagingTimes.size() ? upper_tau_idx - 1 : upper_tau_idx);

                        x_G_0(0) = (_allanVariance.at(s).at(d).at(idx) - sigma_min) / (x_G_0(1) * _gamma_sigma * _gamma_sigma);
                        if (x_G_0(0) < 0. || x_G_0(1) < 0.)
                        {
                            x_G_0 = Eigen::Vector2d::Zero();
                        }
                    }
                    else // compute S_G and tau_G without random walk noise
                    {
                        if (_averagingFactorCount > _averagingFactorsPerDecade && _biasInstability.at(s).at(d) != 0)
                        {
                            x_G_0(1) = _biasInstabilityTau.at(s).at(d) / _gamma_tau;

                            x_G_0(0) = (pow(_biasInstability.at(s).at(d), 2) - _S_N.at(s).at(d) / _biasInstabilityTau.at(s).at(d)) / (x_G_0(1) * _gamma_sigma * _gamma_sigma);
                            if (x_G_0(0) < 0. || x_G_0(1) < 0.)
                            {
                                x_G_0 = Eigen::Vector2d::Zero();
                            }
                        }
                        else
                        {
                            x_G_0 = Eigen::Vector2d::Zero();
                        }
                    }

                    // iterative least squares
                    if (x_G_0(0) < 0. || x_G_0(1) < 0.)
                    {
                        double max_dx = 0;

                        unsigned int iterations = 0;
                        unsigned int number_of_iterations = 100;

                        std::vector<double> taus_G;

                        do
                        {
                            // convert to Eigen data types
                            Eigen::Map<Eigen::VectorXd> y_G(avar_G.data(), m_G);
                            Eigen::Map<Eigen::VectorXd> p_G(weights_G.data(), m_G);

                            Eigen::VectorXd A_S_G;
                            Eigen::VectorXd A_tau_G;

                            Eigen::Matrix<double, Eigen::Dynamic, 2> A_G;

                            Eigen::VectorXd y_0;

                            // compute design matrix and residuals
                            for (size_t k = 0; k < static_cast<unsigned long>(m_G); k++)
                            {
                                long eigen_k = static_cast<long>(k);
                                A_G(eigen_k, 0) = x_G_0(1) / taus_G.at(k)
                                                  * (1. - x_G_0(1) / (2. * taus_G.at(k)) * (3. - 4. * exp(-taus_G.at(k) / x_G_0(1)) + exp(-2. * taus_G.at(k) / x_G_0(1))));
                                A_G(eigen_k, 1) = x_G_0(0) / taus_G.at(k)
                                                  * (2. * x_G_0(1) - 9. * x_G_0(1) * x_G_0(1) / (2. * taus_G.at(k)) + (6. * x_G_0(1) * x_G_0(1) / taus_G.at(k) + 2. * x_G_0(1)) * exp(-taus_G.at(k) / x_G_0(1)) - (3. * x_G_0(1) * x_G_0(1) / (2. * taus_G.at(k)) + x_G_0(1)) * exp(-2. * taus_G.at(k) / x_G_0(1)));

                                y_0(eigen_k) = computeCorrelatedNoiseAllanVariance(x_G_0(0), x_G_0(1), taus_G.at(k));
                            }

                            Eigen::VectorXd dy = y_G - y_0;

                            Eigen::Vector2d dx = (A_G.transpose() * p_G.asDiagonal() * A_G).inverse() * A_G.transpose() * p_G.asDiagonal() * dy;

                            max_dx = dx.norm();

                            x_G_0 += dx;

                            iterations++;
                        } while (max_dx > 1e-10 && iterations < number_of_iterations);
                    }

                    _S_G.at(s).at(d) = x_G_0(0) < 0. ? 0. : x_G_0(0);
                    _tau_G.at(s).at(d) = x_G_0(1) < 0. ? 0. : x_G_0(1);
                }
                else
                {
                    _S_G.at(s).at(d) = 0;
                    _tau_G.at(s).at(d) = 0;
                }
            }
            else
            {
                _S_G.at(s).at(d) = 0;
                _tau_G.at(s).at(d) = 0;
            }

            // compute estimated Allan Deviation
            _estimatedAllanDeviation.at(s).at(d).resize(_averagingFactorCount, 0.);

            for (size_t k = 0; k < _averagingFactorCount; k++)
            {
                _estimatedAllanDeviation.at(s).at(d).at(k) = sqrt(_S_N.at(s).at(d) / _averagingTimes.at(k)
                                                                  + _S_K.at(s).at(d) / 3. * _averagingTimes.at(k)
                                                                  + computeCorrelatedNoiseAllanVariance(_S_G.at(s).at(d), _tau_G.at(s).at(d), _averagingTimes.at(k)));
            }
        }
    }
}

double NAV::AllanDeviation::computeCorrelatedNoiseAllanVariance(double S_G, double tau_G, double tau)
{
    if (S_G == 0. || tau_G == 0.)
    {
        return 0;
    }
    else
    {
        return S_G * tau_G * tau_G / tau * (1 - tau_G / (2 * tau) * (3 - 4 * exp(-tau / tau_G) + exp(-2 * tau / tau_G)));
    }
}