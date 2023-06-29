// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AdevOutput.hpp
/// @brief Allan Deviation Output
/// @author M. Seyfried
/// @date 2023-06-29

#pragma once

#include "NodeData/NodeData.hpp"

#include <array>

#include "util/Eigen.hpp"

namespace NAV
{
/// Allan Deviation Parameters storage class
class AdevOutput : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "AdevOutput";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// Accelerometer
    /// The White Noise Coefficient N² [m²/s³]
    std::optional<Eigen::Vector3d> accelWhiteNoiseCoefficient;
    /// The Random Walk Coefficient K² [m²/s⁵]
    std::optional<Eigen::Vector3d> accelRandomWalkCoefficient;
    /// The Correlated Noise Amplitude G^2 [m²/s⁵]
    std::optional<Eigen::Vector3d> accelCorrelatedNoiseAmplitude;
    /// The Correlated Noise Correlation Time τ_G [s]
    std::optional<Eigen::Vector3d> accelCorrelatedNoiseCorrelationTime;
    /// The Bias Instability [m/s^2]
    std::optional<Eigen::Vector3d> accelBiasInstability;
    /// The τ of the Bias Instability [s]
    std::optional<Eigen::Vector3d> accelBiasInstabilityTau;

    /// Gyroscope
    /// The White Noise Coefficient N² [rad²/s]
    std::optional<Eigen::Vector3d> gyroWhiteNoiseCoefficient;
    /// The Random Walk Coefficient K² [rad²/s³]
    std::optional<Eigen::Vector3d> gyroRandomWalkCoefficient;
    /// The Correlated Noise Amplitude G^2 [rad²/s³]
    std::optional<Eigen::Vector3d> gyroCorrelatedNoiseAmplitude;
    /// The Correlated Noise Correlation Time τ_G [s]
    std::optional<Eigen::Vector3d> gyroCorrelatedNoiseCorrelationTime;
    /// The Bias Instability [rad/s]
    std::optional<Eigen::Vector3d> gyroBiasInstability;
    /// The τ of the Bias Instability [s]
    std::optional<Eigen::Vector3d> gyroBiasInstabilityTau;
};
} // namespace NAV