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
/// @brief Demonstrates the basic GUI functionality of nodes
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

    /// Cumulative Sums of accelerometer and gyroscope data
    std::vector<Eigen::Vector3d> _accelCumSum{ Eigen::Vector3d::Zero() };
    std::vector<Eigen::Vector3d> _gyroCumSum{ Eigen::Vector3d::Zero() };

    std::vector<double> _averagingFactors;
    std::vector<double> _observationCount;

    std::array<std::vector<double>, 3> _accelAllanSum;
    std::array<std::vector<double>, 3> _gyroAllanSum;

    std::array<std::vector<double>, 3> _accelAllanVariance;
    std::array<std::vector<double>, 3> _gyroAllanVariance;

    Eigen::Vector3d _accelTempSum;
    Eigen::Vector3d _gyroTempSum;

    unsigned int _cumSumLength{ 1 };

    double _averagingFactorsPerDecade{ 100 };

    unsigned int _nextAveragingFactorExponent{ 1 };

    unsigned int _nextAveragingFactor{ 1 };
};

} // namespace NAV