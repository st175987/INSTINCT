// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Gravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

#include <Eigen/Dense>

namespace NAV
{
/// Available Gravitation Models
enum class GravitationModel : int
{
    None,         ///< Gravity Model turned off
    WGS84,        ///< World Geodetic System 1984
    WGS84_Skydel, ///< World Geodetic System 1984 implemented by the Skydel Simulator // FIXME: Remove after Skydel uses the same as Instinct
    Somigliana,   ///< Somigliana gravity model
    EGM96,        ///< Earth Gravitational Model 1996
    COUNT,        ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] gravitationModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GravitationModel gravitationModel);

/// @brief Shows a ComboBox to select the gravitation model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] gravitationModel Reference to the gravitation model to select
bool ComboGravitationModel(const char* label, GravitationModel& gravitationModel);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth)
/// @param[in] lla_position [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] gravitationModel Gravitation model to use
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
[[nodiscard]] Eigen::Vector3d n_calcGravitation(const Eigen::Vector3d& lla_position, GravitationModel gravitationModel = GravitationModel::EGM96);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] Eigen::Vector3d n_calcGravitation_SomiglianaAltitude(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential. However, the north component of the centrifugal acceleration is neglected in order to match the implementation of Skydel's 'ImuPlugin'
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Skydel API plug-in 'skydel_plugin/source/library/inertial_math/Sources/source/gravity.cpp'
[[nodiscard]] Eigen::Vector3d n_calcGravitation_WGS84_Skydel(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential.
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See 'INS-Projects/INSTINCT/SpecificLiterature/GravityPotentialWGS84' in NC folder (eq. (3) derived after 'r')
[[nodiscard]] Eigen::Vector3d n_calcGravitation_WGS84(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the EGM96 spherical harmonic model (up to order 10)
/// @param[in] lla_position [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] ndegree Degree of the EGM96 (1 <= ndegree <= 10)
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Groves (2013) Chapter 2.4.3 and 'GUT User Guide' (2018) Chapter 7.4
[[nodiscard]] Eigen::Vector3d n_calcGravitation_EGM96(const Eigen::Vector3d& lla_position, size_t ndegree = 10);

} // namespace NAV
