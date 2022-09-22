// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuObs.hpp
/// @brief Parent Class for all IMU Observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "NodeData/NodeData.hpp"

#include "ImuPos.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// IMU Observation storage class
class ImuObs : public NodeData
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObs(const ImuPos& imuPos)
        : imuPos(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ImuObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;

    /// The system time since startup measured in [nano seconds].
    std::optional<uint64_t> timeSinceStartup;

    /// The IMU magnetic field measured in units of [Gauss], given in the platform frame.
    std::optional<Eigen::Vector3d> magUncompXYZ;
    /// The IMU acceleration measured in units of [m/s^2], given in the platform frame.
    std::optional<Eigen::Vector3d> accelUncompXYZ;
    /// The IMU angular rate measured in units of [rad/s], given in the platform frame.
    std::optional<Eigen::Vector3d> gyroUncompXYZ;

    /// The compensated magnetic field measured in units of [Gauss], and given in the platform frame.
    std::optional<Eigen::Vector3d> magCompXYZ;
    /// The compensated acceleration measured in units of [m/s^2], and given in the platform frame.
    std::optional<Eigen::Vector3d> accelCompXYZ;
    /// The compensated angular rate measured in units of [rad/s], and given in the platform frame.
    std::optional<Eigen::Vector3d> gyroCompXYZ;

    /// The IMU temperature measured in units of [Celsius].
    std::optional<double> temperature = 0.0;
};

} // namespace NAV
