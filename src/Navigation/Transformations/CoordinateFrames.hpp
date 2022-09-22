// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CoordinateFrames.hpp
/// @brief Transformation collection
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-08
///
/// Coordinate Frames:
/// - Inertial frame (i-frame)
///     O_i: Earth center
///     x_i: Direction to Vernal equinox
///     y_i: In equatorial plane, complementing to Right-Hand-System
///     z_i: Vertical on equatorial plane (North)
/// - Earth-centered-Earth-fixed frame (e-frame)
///     O_e: Earth center of mass
///     x_e: Direction to Greenwich meridian (longitude = 0°)
///     y_e: In equatorial plane, complementing to Right-Hand-System
///     z_e: Vertical on equatorial plane (North)
/// - Local Navigation frame (n-frame)
///     O_n: Vehicle center of mass
///     x_n: "North"
///     y_n: Right-Hand-System ("East")
///     z_n: Earth center ("Down")
/// - Body frame (b-frame)
///     O_b: Vehicle center of mass
///     x_b: Roll-axis ("Forward")
///     y_b: Pitch-axis ("Right")
///     z_b: Yaw-axis ("Down")
/// - Platform frame (p-frame)
///     O_b: Center of IMU
///     x_b: X-Axis Accelerometer/Gyrometer
///     y_b: Y-Axis Accelerometer/Gyrometer
///     z_b: Z-Axis Accelerometer/Gyrometer

#pragma once

#include <cmath>
#include "util/Eigen.hpp"

#include "Navigation/Constants.hpp"

namespace NAV
{
// TODO: Reenable concepts as soon as they become supported by Apple Clang
namespace Concepts
{
// // Only allow numerical Types
// template<typename T>
// concept Numerical = std::is_arithmetic<T>::value;
} // namespace Concepts

namespace trafo
{
/// @brief Converts the quaternion to Euler rotation angles with rotation sequence ZYX
/// @param[in] q Quaternion to convert
/// @return [angleX, angleY, angleZ]^T vector in [rad]. The returned angles are in the ranges (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
[[nodiscard]] Eigen::Vector3d quat2eulerZYX(const Eigen::Quaterniond& q);

/// @brief Quaternion for rotations from inertial to Earth-centered-Earth-fixed frame
/// @param[in] time Time (t - t0)
/// @param[in] omega_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond e_Quat_i(double time, double omega_ie = InsConst::omega_ie);

/// @brief Quaternion for rotations from Earth-centered-Earth-fixed to inertial frame
/// @param[in] time Time (t - t0)
/// @param[in] omega_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond i_Quat_e(double time, double omega_ie = InsConst::omega_ie);

/// @brief Quaternion for rotations from navigation to Earth-fixed frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond e_Quat_n(double latitude, double longitude);

/// @brief Quaternion for rotations from Earth-fixed to navigation frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond n_Quat_e(double latitude, double longitude);

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond n_Quat_b(double roll, double pitch, double yaw);

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] rollPitchYaw Roll, Pitch, Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond n_Quat_b(const Eigen::Vector3d& rollPitchYaw);

/// @brief Quaternion for rotations from navigation to body frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond b_Quat_n(double roll, double pitch, double yaw);

/// @brief Quaternion for rotations from navigation to body frame
/// @param[in] rollPitchYaw Roll, Pitch, Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond b_Quat_n(const Eigen::Vector3d& rollPitchYaw);

/// @brief Quaternion for rotations from platform to body frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond b_Quat_p(double mountingAngleX, double mountingAngleY, double mountingAngleZ);

/// @brief Quaternion for rotations from body to platform frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond p_Quat_b(double mountingAngleX, double mountingAngleY, double mountingAngleZ);

/// @brief Converts ECEF coordinates into local NED coordinates
/// @param[in] e_position ECEF coordinates in [m] to convert
/// @param[in] lla_position_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return [x_N, x_E, x_D]^T Local NED coordinates in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
[[nodiscard]] Eigen::Vector3d ecef2ned(const Eigen::Vector3d& e_position, const Eigen::Vector3d& lla_position_ref);

/// @brief Converts local NED coordinates into ECEF coordinates
/// @param[in] n_position NED coordinates in [m] to convert
/// @param[in] lla_position_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return ECEF position in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
[[nodiscard]] Eigen::Vector3d ned2ecef(const Eigen::Vector3d& n_position, const Eigen::Vector3d& lla_position_ref);

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using WGS84
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d lla2ecef_WGS84(const Eigen::Vector3d& lla_position);

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using GRS90
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d lla2ecef_GRS80(const Eigen::Vector3d& lla_position);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using WGS84
/// @param[in] e_position Vector with coordinates in ECEF frame
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
[[nodiscard]] Eigen::Vector3d ecef2lla_WGS84(const Eigen::Vector3d& e_position);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using GRS90
/// @param[in] e_position Vector with coordinates in ECEF frame in [m]
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
[[nodiscard]] Eigen::Vector3d ecef2lla_GRS80(const Eigen::Vector3d& e_position);

/// @brief Converts PZ-90.11 coordinates to WGS84 coordinates
/// @param[in] pz90_pos Position in PZ-90.11 coordinates
/// @return Position in WGS84 coordinates
/// @note See \cite PZ-90.11 PZ-90.11 Reference Document Appendix 4, p.34f
[[nodiscard]] Eigen::Vector3d pz90toWGS84_pos(const Eigen::Vector3d& pz90_pos);

/// @brief Converts PZ-90.11 vectors to WGS84 frame
/// @param[in] pz90 Vector in PZ-90.11 frame
/// @param[in] pz90_pos Position in PZ-90.11 frame (needed for calculation)
/// @return Vector in WGS84 frame
[[nodiscard]] Eigen::Vector3d pz90toWGS84(const Eigen::Vector3d& pz90, const Eigen::Vector3d& pz90_pos);

/// @brief Converts spherical Earth-centered-Earth-fixed coordinates into cartesian coordinates
/// @param[in] position_s Position in spherical coordinates to convert
/// @param[in] elevation Elevation in [rad]
/// @param[in] azimuth Azimuth in [rad]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d sph2ecef(const Eigen::Vector3d& position_s, const double& elevation, const double& azimuth);

} // namespace trafo

} // namespace NAV
