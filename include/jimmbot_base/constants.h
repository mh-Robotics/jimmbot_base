/**
 * @file constants.h
 * @brief This file defines various constants that are used throughout the
 * jimmbot_base namespace, including control frequencies, maximum wheel speeds,
 * and topic names for feedback, commands, and camera tilt information.
 * @version 0.1
 * @date 2023-03-01
 *
 * @copyright Copyright (c) 2020-2023, mhRobotics, Inc. All rights reserved.
 * @license This project is released under the BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef JIMMBOT_BASE_CONSTANTS_H_
#define JIMMBOT_BASE_CONSTANTS_H_

namespace jimmbot_base {

constexpr auto kFirst{0};
constexpr auto kSecond{1};

// Control frequency constants
constexpr auto kControlFrequencyKey{"control_frequency"};
constexpr auto kDefaultControlFrequency{10.0};

// Maximum wheel speed constants
constexpr auto kMaxWheelSpeedKey{"max_wheel_speed"};
constexpr auto kDefaultMaxAllowedWheelSpeed{2.5};

// Command topic constants
constexpr auto kCommandFrameIdKey{"command_frame_id"};
constexpr auto kDefaultCommandFrameId{"/jimmbot/hw/cmd"};
constexpr auto kDefaultCommandTopic{"command/can_msg"};

// Feedback topic constants
constexpr auto kDefaultFeedbackTopic{"feedback/can_msg"};
constexpr auto kDefaultExtendedDataTopic{"extn_data"};

// Camera tilt topic constants
constexpr auto kDefaultFrontCameraTiltTopic{"camera_tilt_front"};
constexpr auto kDefaultBackCameraTiltTopic{"camera_tilt_back"};

// Wheel constants
constexpr auto kLeftWheelFrontKey{"left_wheel_front"};
constexpr auto kDefaultLeftWheelFront{
    "wheel_axis_front_left_to_wheel_front_left"};
constexpr auto kLeftWheelBackKey{"left_wheel_back"};
constexpr auto kDefaultLeftWheelBack{"wheel_axis_back_left_to_wheel_back_left"};
constexpr auto kRightWheelFrontKey{"right_wheel_front"};
constexpr auto kDefaultRightWheelFront{
    "wheel_axis_front_right_to_wheel_front_right"};
constexpr auto kRightWheelBackKey{"right_wheel_back"};
constexpr auto kDefaultRightWheelBack{
    "wheel_axis_back_right_to_wheel_back_right"};

}  // namespace jimmbot_base

#endif  // JIMMBOT_BASE_CONSTANTS_H_
