/**
 * @file can_packt.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for packing and unpacking compressed CAN messages.
 * @version 0.1
 * @date 2021-10-09
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
 *
 */
#include <jimmbot_msgs/CanFrame.h>  // for jimmbot_msg::CanFrame

#ifndef JIMMBOT_BASE_CAN_PACKT_H_
#define JIMMBOT_BASE_CAN_PACKT_H_

#define CAN_MAX_DLEN 8

typedef struct WheelStatus {
  int _command = {0};
  int _effort = {0};
  double _position = {0};
  int _rpm = {0};
  double _velocity = {0};
} wheel_status_t;

/**
 * @brief Defines a bit-field struct to represent the compressed motor status
 * data
 */
typedef struct __attribute__((packed)) {
  uint32_t command_id : 8; /**< Command ID */
  uint32_t effort : 12;    /**< Effort */
  int32_t position : 19;   /**< Position: 1 sign bit + 18 bits for magnitude */
  uint32_t rpm : 10;       /**< RPM */
  int32_t velocity : 23;   /**< Velocity: 1 sign bit + 22 bits for magnitude */
} compressed_wheel_status_t;

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
   * @brief Packs a compressed wheel status data structure into a CAN frame
   *
   * @tparam inType the input data type (must be a wheel_status_t)
   * @tparam outType the output data type (must be a can_frame_t)
   * @param data the input data to pack
   * @return the packed CAN frame
   */
  template <typename inType, typename outType>
  outType PackCompressed(const inType &data) {
    static_assert(sizeof(inType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    jimmbot_msgs::CanFrame canFrame;
    canFrame.id = transmit_id_;
    canFrame.dlc = CAN_MAX_DLEN;

    ::memcpy(canFrame.data, &data, sizeof(inType));

    return canFrame;
  }

  /**
   * @brief Unpacks a compressed wheel status CAN frame into a wheel status data
   * structure
   *
   * @tparam inType the input data type (must be a can_frame_t)
   * @tparam outType the output data type (must be a wheel_status_t)
   * @param msg the CAN frame to unpack
   * @return the unpacked wheel status data structure
   */
  template <typename inType, typename outType>
  outType UnpackCompressed(const inType &can_frame) {
    static_assert(sizeof(outType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    outType data;
    ::memcpy(&data, can_frame.data, sizeof(outType));

    return data;
  }

 private:
  uint8_t transmit_id_{0x00}; /**< The ID of the transmitter node */
  uint8_t receive_id_{0x00};  /**< The ID of the receiver node */
};

/**
 * @brief Specialization of the PackCompressed template function for packing a
 * wheel_status_t into a can_frame_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param wheel_status The input data to pack.
 * @return The packed data as a CAN frame.
 */
template <>
inline jimmbot_msgs::CanFrame
CanPackt::PackCompressed<wheel_status_t, jimmbot_msgs::CanFrame>(
    const wheel_status_t &wheel_status) {
  jimmbot_msgs::CanFrame canFrame;
  canFrame.id = transmit_id_;
  canFrame.dlc = CAN_MAX_DLEN;

  // Compress the motor status data into a bit-field struct
  compressed_wheel_status_t compressed_status;
  compressed_status.command_id = wheel_status._command;
  compressed_status.effort = wheel_status._effort & 0xFFF;
  compressed_status.position =
      static_cast<int32_t>(wheel_status._position * 100);
  compressed_status.rpm = wheel_status._rpm & 0x3FF;
  compressed_status.velocity =
      static_cast<int32_t>(wheel_status._velocity * 100);

  // Copy the compressed data into the CAN frame
  ::memcpy(canFrame.data.c_array(), &compressed_status,
           sizeof(compressed_wheel_status_t));

  return canFrame;
}

/**
 * @brief Specialization of the PackCompressed template function for unpacking a
 * jimmbot_msgs::CanFrame into a wheel_status_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param jimmbot_msgs::CanFrame The input data to pack.
 * @return The packed data as a wheel status data structure.
 */
template <>
inline wheel_status_t
CanPackt::UnpackCompressed<jimmbot_msgs::CanFrame, wheel_status_t>(
    const jimmbot_msgs::CanFrame &can_frame) {
  wheel_status_t wheel_status;

  // Extract the compressed data from the CAN frame
  compressed_wheel_status_t compressed_status;
  ::memcpy(&compressed_status, can_frame.data.data(),
           sizeof(compressed_wheel_status_t));

  // Unpack the compressed data into the motor status struct
  wheel_status._command = compressed_status.command_id;
  wheel_status._effort = compressed_status.effort;
  wheel_status._position =
      static_cast<double>(compressed_status.position) / 100;
  wheel_status._rpm = compressed_status.rpm;
  wheel_status._velocity =
      static_cast<double>(compressed_status.velocity) / 100;

  return wheel_status;
}
#endif  // JIMMBOT_BASE_CAN_PACKT_H_
