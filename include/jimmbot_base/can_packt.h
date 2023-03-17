/**
 * @file can_packt.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for packing and unpacking compressed CAN messages.
 * @version 0.2
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
#include <jimmbot_msgs/CanFrame.h>  // for jimmbot_msg::CanFrame

#include "constants.h"  // for jimmbot_base::k*

#ifndef JIMMBOT_BASE_CAN_PACKT_H_
#define JIMMBOT_BASE_CAN_PACKT_H_

using WheelStatus = struct WheelStatus {
  int command_id{0};
  double effort{0.0};
  double position{0.0};
  int rpm = {0};
  double velocity{0.0};
};

/**
 * @brief Defines a bit-field struct to represent the compressed motor status
 * data
 */
// @todo(jimmyhalimi): Old struct was 9 Byte, the new one is 7. Check the
// warning for offset of packed bit-field.
using CompressedWheelStatus =
    struct __attribute__((packed)) CompressedWheelStatus {
  uint8_t command_id : 7; /**< Command ID (7 bits), Range 0 to 127 */
  int8_t effort : 7;      /**< Effort (7 bits), Range -64 to 63 */
  int32_t position : 22;  /**< Position (1 sign bit + 21 bits for magnitude),
                             Range -2,097,151 to 2,097,151. */
  uint16_t rpm : 10;      /**< RPM (10 bits), Range 0 to 1023  */
  int8_t
      velocity : 8; /**< Velocity (signed 8-bit magnitude), Range -127 to 127 */
};

/**
 * @brief A class for packing and unpacking compressed CAN messages
 */
class CanPackt {
 public:
  /**
   * @brief Construct a new Can Packt object
   *
   * @param transmit_id the ID of the transmitter node
   * @param receive_id the ID of the receiver node
   */
  CanPackt(uint8_t transmit_id, uint8_t receive_id)
      : transmit_id_(transmit_id), receive_id_(receive_id){};

  /**
   * @brief Returns the transmit ID value.
   *
   * @return The transmit ID value.
   */
  [[nodiscard]] inline uint8_t TransmitId() const { return transmit_id_; };

  /**
   * @brief Returns the receive ID value.
   *
   * @return The receive ID value.
   */
  [[nodiscard]] inline uint8_t ReceiveId() const { return receive_id_; };

  /**
   * @brief Packs a compressed wheel status data structure into a CAN frame
   *
   * @tparam inType the input data type (must be a wheel_status_t)
   * @tparam outType the output data type (must be a can_frame_t)
   * @param data the input data to pack
   * @return the packed CAN frame
   */
  template <typename inType, typename outType>
  outType PackCompressed(const inType& wheel_status) const {
    static_assert(sizeof(inType) <= jimmbot_base::kCanMaxDLen,
                  "Struct is larger than CAN message data field size");

    jimmbot_msgs::CanFrame can_frame;
    can_frame.id = transmit_id_;
    can_frame.dlc = jimmbot_base::kCanMaxDLen;

    std::memcpy(can_frame.data, &wheel_status, sizeof(inType));

    return can_frame;
  }

  /**
   * @brief Unpacks a compressed wheel status CAN frame into a wheel status
   * data structure
   *
   * @tparam inType the input data type (must be a can_frame_t)
   * @tparam outType the output data type (must be a wheel_status_t)
   * @param msg the CAN frame to unpack
   * @return the unpacked wheel status data structure
   */
  template <typename inType, typename outType>
  outType UnpackCompressed(const inType& can_frame) const {
    static_assert(sizeof(outType) <= jimmbot_base::kCanMaxDLen,
                  "Struct is larger than CAN message data field size");

    outType data;
    std::memcpy(&data, can_frame.data, sizeof(outType));

    return data;
  }

 private:
  uint8_t transmit_id_{0x00}; /**< The ID of the transmitter node */
  uint8_t receive_id_{0x00};  /**< The ID of the receiver node */
};

/**
 * @brief Specialization of the PackCompressed template function for packing
 * a wheel_status_t into a can_frame_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param wheel_status The input data to pack.
 * @return The packed data as a CAN frame.
 */
template <>
inline jimmbot_msgs::CanFrame
CanPackt::PackCompressed<WheelStatus, jimmbot_msgs::CanFrame>(
    const WheelStatus& wheel_status) const {
  static_assert(sizeof(CompressedWheelStatus) <= jimmbot_base::kCanMaxDLen,
                "Struct is larger than CAN message data field size");

  jimmbot_msgs::CanFrame can_frame;
  can_frame.id = transmit_id_;
  can_frame.dlc = jimmbot_base::kCanMaxDLen;

  // Compress the motor status data into a bit-field struct
  CompressedWheelStatus compressed_status;
  compressed_status.command_id = static_cast<int8_t>(wheel_status.command_id);
  compressed_status.effort = static_cast<int>(wheel_status.effort) & 0xFFF;
  compressed_status.position =
      static_cast<int32_t>(wheel_status.position * 100);
  compressed_status.rpm = static_cast<uint16_t>(wheel_status.rpm) & 0x3FF;
  compressed_status.velocity = static_cast<int8_t>(wheel_status.velocity * 20);

  // Copy the compressed data into the CAN frame
  std::memcpy(can_frame.data.c_array(), &compressed_status,
              sizeof(CompressedWheelStatus));

  return can_frame;
}

/**
 * @brief Specialization of the PackCompressed template function for
 * unpacking a jimmbot_msgs::CanFrame into a WheelStatus
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param jimmbot_msgs::CanFrame The input data to pack.
 * @return The packed data as a wheel status data structure.
 */
template <>
inline WheelStatus
CanPackt::UnpackCompressed<jimmbot_msgs::CanFrame, WheelStatus>(
    const jimmbot_msgs::CanFrame& can_frame) const {
  static_assert(
      sizeof(jimmbot_msgs::CanFrame::data) <= jimmbot_base::kCanMaxDLen,
      "Struct is larger than CAN message data field size");

  WheelStatus wheel_status;

  // Extract the compressed data from the CAN frame
  CompressedWheelStatus compressed_status;
  std::memcpy(&compressed_status, can_frame.data.data(),
              sizeof(CompressedWheelStatus));

  // Unpack the compressed data into the motor status struct
  wheel_status.command_id = static_cast<int>(compressed_status.command_id);
  wheel_status.effort = static_cast<double>(compressed_status.effort);
  wheel_status.position = static_cast<double>(compressed_status.position) / 100;
  wheel_status.rpm = static_cast<int>(compressed_status.rpm);
  wheel_status.velocity = static_cast<double>(compressed_status.velocity) / 20;

  return wheel_status;
}
#endif  // JIMMBOT_BASE_CAN_PACKT_H_
