// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Imu.hpp
/// @brief Abstract IMU Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuPos.hpp"

namespace NAV
{
/// Abstract IMU Class
class Imu : public Node
{
  public:
    /// @brief Destructor
    ~Imu() override = default;
    /// @brief Copy constructor
    Imu(const Imu&) = delete;
    /// @brief Move constructor
    Imu(Imu&&) = delete;
    /// @brief Copy assignment operator
    Imu& operator=(const Imu&) = delete;
    /// @brief Move assignment operator
    Imu& operator=(Imu&&) = delete;

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// Position and rotation information for conversion from platform to body frame
    [[nodiscard]] const ImuPos& imuPosition() const { return _imuPos; }

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the node
    explicit Imu(std::string name);

    /// Position and rotation information for conversion from platform to body frame
    ImuPos _imuPos;
};

} // namespace NAV
