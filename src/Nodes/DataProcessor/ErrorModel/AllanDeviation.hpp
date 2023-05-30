// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AllanDeviation.hpp
/// @brief Computes Allan Deviation
/// @author M. Seyfried
/// @date 2023-03-29

#pragma once

#include "internal/Node/Node.hpp"

#include "implot.h"

#include "util/Eigen.hpp"
#include <array>
#include <vector>

namespace NAV
{
/// @brief Computes Allan Deviation of IMU Observations
class AllanDeviation : public Node
{
  public:
    /// @brief Default constructor
    AllanDeviation();
    /// @brief Destructor
    ~AllanDeviation() override;
    /// @brief Copy constructor
    AllanDeviation(const AllanDeviation&) = delete;
    /// @brief Move constructor
    AllanDeviation(AllanDeviation&&) = delete;
    /// @brief Copy assignment operator
    AllanDeviation& operator=(const AllanDeviation&) = delete;
    /// @brief Move assignment operator
    AllanDeviation& operator=(AllanDeviation&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_ADEV_OUTPUT = 0; ///< @brief Flow (AdevOutput)
    constexpr static size_t INPUT_PORT_INDEX_IMU_OBS = 0;      ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Sensor Data
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveImuObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    int _valueObject; ///< Value which is represented over the Object pin

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    void computeSlopes();

    void estimateNoiseParameters();

    /// sampling interval
    double _samplingInterval;

    InsTime _startingInsTime;

    /// Cumulative Sums of accelerometer and gyroscope data
    std::vector<Eigen::Vector3d> _accelCumSum{ Eigen::Vector3d::Zero() };
    std::vector<Eigen::Vector3d> _gyroCumSum{ Eigen::Vector3d::Zero() };

    /// temporary variable for computation of Allan Sum
    Eigen::Vector3d _accelTempSum;
    Eigen::Vector3d _gyroTempSum;

    /// Allan Variance precursor
    std::array<std::vector<double>, 3> _accelAllanSum;
    std::array<std::vector<double>, 3> _gyroAllanSum;

    /// Allan Variance of accelerometer and gyroscope data
    std::array<std::vector<double>, 3> _accelAllanVariance;
    std::array<std::vector<double>, 3> _gyroAllanVariance;

    /// Allan Deviation of accelerometer and gyroscope data
    std::array<std::vector<double>, 3> _accelAllanDeviation;
    std::array<std::vector<double>, 3> _gyroAllanDeviation;

    /// Slope of Allan Variance of accelerometer and gyroscope data
    std::array<std::vector<double>, 3> _accelSlope;
    std::array<std::vector<double>, 3> _gyroSlope;

    /// averaging factors (n) used for Allan Variance computation
    std::vector<double> _averagingFactors;

    /// averaging times (τ)
    std::vector<double> _averagingTimes;

    /// number of averaging factors
    unsigned int _averagingFactorCount{ 0 };

    /// number of observations for each τ
    std::vector<double> _observationCount;

    /// number of IMU observations / length of cumulative sums
    unsigned int _imuObsCount{ 0 };

    /// number of averaging factors per decade
    double _averagingFactorsPerDecade{ 100 };

    /// next averaging factor to be appended to _averagingFactors
    unsigned int _nextAveragingFactor{ 1 };

    /// exponent of next averaging factor
    unsigned int _nextAveragingFactorExponent{ 1 };

    /// multiplication factor for simple confidence
    std::vector<double> _confidenceMultiplicationFactor;

    /// Confidence of Allan Deviation of accelerometer and gyroscope data
    std::array<std::array<std::vector<double>, 2>, 3> _accelAllanDeviationConfidenceIntervals;
    std::array<std::array<std::vector<double>, 2>, 3> _gyroAllanDeviationConfidenceIntervals;

    bool _updateLast{ false };

    std::array<double, 3> _accel_S_N;

    std::array<std::vector<double>, 3> _accelEstimatedAllanDeviation;
};

} // namespace NAV
