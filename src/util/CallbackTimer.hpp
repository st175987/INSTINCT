// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CallbackTimer.hpp
/// @brief Starts a Periodic Timer
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-15

#pragma once

#include <functional>
#include <thread>
#include <atomic>

/// @brief Manages a thread which calls a specified function at a specified interval
class CallbackTimer
{
  public:
    /// @brief Default constructor
    CallbackTimer() = default;
    /// @brief Copy constructor
    CallbackTimer(const CallbackTimer&) = delete;
    /// @brief Move constructor
    CallbackTimer(CallbackTimer&&) = delete;
    /// @brief Copy assignment operator
    CallbackTimer& operator=(const CallbackTimer&) = delete;
    /// @brief Move assignment operator
    CallbackTimer& operator=(CallbackTimer&&) = delete;
    /// @brief Destructor
    ~CallbackTimer();

    /// @brief Stops the Timer
    void stop();

    /// @brief Starts the timer
    /// @param[in] interval Interval in [ms] when to trigger the callback
    /// @param[in] func Function to call
    /// @param[in, out] userData User Data which will be passed to the callback function
    void start(int interval, const std::function<void(void*)>& func, void* userData);

    /// @brief Set the Interval of the timer
    /// @param[in] interval Interval in [ms] when to trigger the callback
    void setInterval(int interval);

    /// @brief Checks if the timer is currently running
    /// @return True if the timer is running
    [[nodiscard]] bool is_running() const noexcept;

  private:
    /// @brief Interval in which the timer is triggered
    std::atomic<int> _interval{ 0 };
    /// @brief Flag whether the timer should execute
    std::atomic<bool> _execute{ false };
    /// @brief Thread object which triggers the timer
    std::thread _thd;
};