// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <regex>
#include <string>

#include <jaguar4x4_comms_msgs/msg/motor_board.hpp>

#include "jaguar4x4_comms/MotorParse.h"
#include "jaguar4x4_comms/Utils.h"

static double adToTemperature(uint16_t adValue)
{
  static const double resTable[25] = {
    114660, 84510, 62927, 47077, 35563,
    27119, 20860, 16204, 12683, 10000,
    7942, 6327, 5074, 4103, 3336,
    2724, 2237, 1846, 1530, 1275,
    1068, 899.3, 760.7, 645.2, 549.4
  };
  static const double tempTable[25] = {
    -20, -15, -10, -5, 0,
    5, 10, 15, 20, 25,
    30, 35, 40, 45, 50,
    55, 60, 65, 70, 75,
    80, 85, 90, 95, 100
  };
  static const double FULLAD = 4095;

  // for new temperature sensor
  double tempM = 0;
  double k = (adValue / FULLAD);
  double resValue = 0;

  if (k != 0) {
    resValue = (10000 / k -10000);      //AD value to resistor
  } else {
    resValue = resTable[0];
  }

  int index = -1;
  if (resValue >= resTable[0]) {      //too lower
    tempM = -20;
  } else if (resValue <= resTable[24]) {
    tempM = 100;
  } else {
    for (int i = 0; i < 24; ++i) {
      if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1])) {
        index = i;
        break;
      }
    }
    if (index >= 0) {
      tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
    } else {
      tempM = 0;
    }
  }

  return tempM;
}

MotorTempMsg::MotorTempMsg(uint16_t temp1, uint16_t temp2)
  : AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_temperature),
    motor_temp_adc_1_(temp1), motor_temp_adc_2_(temp2)
{
  motor_temp_1_ = adToTemperature(temp1);
  motor_temp_2_ = adToTemperature(temp2);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorAmpMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("A=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorAmpMsg>(str_to_d(sm[1]) / 10.0,
                                         str_to_d(sm[2]) / 10.0);
  }

  std::cerr << "Motor Amp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncPosMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("C=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncPosMsg>(std::stol(sm[1]),
                                            std::stol(sm[2]));
  }

  std::cerr << "Motor EncPos Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncPosDiffMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("CR=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncPosDiffMsg>(std::stol(sm[1]),
                                                std::stol(sm[2]));
  }

  std::cerr << "Motor EncPosDiff Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnDigitalInputMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("D=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorDigitalInputMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnDigitalOutputMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("DO=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorDigitalOutputMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorPowerMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("P=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorPowerMsg>(std::stol(sm[1]),
                                           std::stol(sm[2]));
  }

  std::cerr << "Motor Power Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorEncVelMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("S=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorEncVelMsg>(std::stol(sm[1]),
                                            std::stol(sm[2]));
  }

  std::cerr << "Motor EncVel Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorBoardTempMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("T=(-?[0-9]*?):(-?[0-9]*?)$"))) {
      return std::make_unique<MotorBoardTempMsg>(str_to_d(sm[1]),
                                                 str_to_d(sm[2]));
  }

  std::cerr << "Motor BoardTemp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorVoltageMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("V=(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorVoltageMsg>(
                                             str_to_d(sm[1])/10.0,
                                             str_to_d(sm[2])/10.0,
                                             str_to_d(sm[3])/1000.0);
  }

  std::cerr << "Motor Voltage Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorTempMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm,
                       std::regex("AI=(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?):(-?[0-9]*?)$"))) {
      return std::make_unique<MotorTempMsg>(std::stoul(sm[3]),
                                            std::stoul(sm[4]));
  }

  std::cerr << "Motor Temp Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorFlagsMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("FF=(-?[0-9]*?)$"))) {
    return std::make_unique<MotorFlagsMsg>(std::stoul(sm[1]));
  }

  std::cerr << "Motor Flags Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

static std::unique_ptr<AbstractMotorMsg> parseAndReturnMotorModeMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("MMOD=(-?[0-9]*?):(-?[0-9]*?)$"))) {
    return std::make_unique<MotorModeMsg>(std::stoul(sm[1]),
                                          std::stoul(sm[2]));
  }

  std::cerr << "Motor Mode Msg failed to parse: '" << msg << "' (" << dumpHex(msg) << ")" << std::endl;

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

std::unique_ptr<AbstractMotorMsg> parseMotorMessage(const std::string& msg)
{
  std::smatch sm;

  // nothing before the "\r"
  if (msg.empty()) {
    return std::unique_ptr<AbstractMotorMsg>(nullptr);
  }

  if (startsWith(msg, "A=")) {
    return parseAndReturnMotorAmpMsg(msg);
  } else if (startsWith(msg, "C=")) {
    return parseAndReturnMotorEncPosMsg(msg);
  } else if (startsWith(msg, "CR=")) {
    return parseAndReturnMotorEncPosDiffMsg(msg);
  } else if (startsWith(msg, "D=")) {
    return parseAndReturnDigitalInputMsg(msg);
  } else if (startsWith(msg, "DO=")) {
    return parseAndReturnDigitalOutputMsg(msg);
  } else if (startsWith(msg, "P=")) {
    return parseAndReturnMotorPowerMsg(msg);
  } else if (startsWith(msg, "S=")) {
    return parseAndReturnMotorEncVelMsg(msg);
  } else if (startsWith(msg, "T=")) {
    return parseAndReturnMotorBoardTempMsg(msg);
  } else if (startsWith(msg, "V=")) {
    return parseAndReturnMotorVoltageMsg(msg);
  } else if (startsWith(msg, "+")) {
    // valid command received
    return std::make_unique<MotorCmdAcceptedMsg>();
  } else if (startsWith(msg, "-")) {
    // INvalid command received
    return std::make_unique<MotorCmdRejectedMsg>();
  } else if (startsWith(msg, "AI=")) {
    return parseAndReturnMotorTempMsg(msg);
  } else if (startsWith(msg, "FF=")) {
    return parseAndReturnMotorFlagsMsg(msg);
  } else if (startsWith(msg, "MMOD=")) {
    return parseAndReturnMotorModeMsg(msg);
  }

  std::cerr << "UNKNOWN MOTOR MESSAGE TYPE '"<< msg << "' (" << dumpHex(msg) << ")" << std::endl;

  return std::unique_ptr<AbstractMotorMsg>(nullptr);
}

void abstractMotorToROS(AbstractMotorMsg* msg, jaguar4x4_comms_msgs::msg::MotorBoard* motor_ref)
{
  switch(msg->getType()) {
  case AbstractMotorMsg::MessageType::motor_amperage:
    {
      MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg);
      motor_ref->amp_1 = motor_amp->motor_amp_1_;
      motor_ref->amp_2 = motor_amp->motor_amp_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::motor_temperature:
    {
      MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg);
      motor_ref->motor_temp_1 = motor_temp->motor_temp_1_;
      motor_ref->motor_temp_2 = motor_temp->motor_temp_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::encoder_position:
    {
      MotorEncPosMsg *motor_enc_pos =
        dynamic_cast<MotorEncPosMsg*>(msg);
      motor_ref->encoder_pos_1 = motor_enc_pos->encoder_pos_1_;
      motor_ref->encoder_pos_2 = motor_enc_pos->encoder_pos_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::encoder_position_diff:
    {
      MotorEncPosDiffMsg *motor_enc_pos_diff =
        dynamic_cast<MotorEncPosDiffMsg*>(msg);
      motor_ref->encoder_diff_1 = motor_enc_pos_diff->pos_diff_1_;
      motor_ref->encoder_diff_2 = motor_enc_pos_diff->pos_diff_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::digital_input:
    {
      MotorDigitalInputMsg *motor_digital_input =
        dynamic_cast<MotorDigitalInputMsg*>(msg);
      motor_ref->digital_input = motor_digital_input->digital_input_;
    }
    break;
  case AbstractMotorMsg::MessageType::digital_output:
    {
      MotorDigitalOutputMsg *motor_digital_output =
        dynamic_cast<MotorDigitalOutputMsg*>(msg);
      motor_ref->digital_output = motor_digital_output->digital_output_;
    }
    break;
  case AbstractMotorMsg::MessageType::motor_power:
    {
      // TODO we've seen this coming out negative... what's up with that?
      // TODO what are the units?  What does this number mean?
      // 25 for power 1 and 0 for power 2 on the arm,
      // -22 and -1 for power on the arm?
      MotorPowerMsg *motor_power =
        dynamic_cast<MotorPowerMsg*>(msg);
      motor_ref->motor_power_1 = motor_power->motor_power_1_;
      motor_ref->motor_power_2 = motor_power->motor_power_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::encoder_velocity:
    {
      MotorEncVelMsg *motor_enc_vel =
        dynamic_cast<MotorEncVelMsg*>(msg);
      motor_ref->encoder_vel_1 = motor_enc_vel->encoder_velocity_1_;
      motor_ref->encoder_vel_2 = motor_enc_vel->encoder_velocity_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::board_temperature:
    {
      MotorBoardTempMsg *motor_board_temp =
        dynamic_cast<MotorBoardTempMsg*>(msg);
      motor_ref->board_temp_1 = motor_board_temp->board_temp_1_;
      motor_ref->board_temp_2 = motor_board_temp->board_temp_2_;
    }
    break;
  case AbstractMotorMsg::MessageType::voltage:
    {
      MotorVoltageMsg *motor_voltage =
        dynamic_cast<MotorVoltageMsg*>(msg);
      motor_ref->volt_main = motor_voltage->bat_voltage_;
      motor_ref->volt_12v = motor_voltage->drv_voltage_;
      motor_ref->volt_5v = motor_voltage->reg_5_voltage_;
    }
    break;
  case AbstractMotorMsg::MessageType::motor_mode:
    {
      MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg);
      motor_ref->motor_control_mode_1 =
        static_cast<std::underlying_type
                    <MotorModeMsg::MotorControlMode>::type>
        (motor_mode->mode_channel_1_);
      motor_ref->motor_control_mode_2 =
        static_cast<std::underlying_type
                    <MotorModeMsg::MotorControlMode>::type>
        (motor_mode->mode_channel_2_);
    }
    break;
  case AbstractMotorMsg::MessageType::motor_flags:
    {
      MotorFlagsMsg *motor_flags =
        dynamic_cast<MotorFlagsMsg*>(msg);
      motor_ref->overheat = motor_flags->overheat_;
      motor_ref->overvoltage = motor_flags->overvoltage_;
      motor_ref->undervoltage = motor_flags->undervoltage_;
      motor_ref->e_short = motor_flags->short_;
      motor_ref->estop = motor_flags->ESTOP_;
    }
    break;
  case AbstractMotorMsg::MessageType::command_accepted:
    {
      motor_ref->last_cmd_ack = true;
    }
    break;
  case AbstractMotorMsg::MessageType::command_rejected:
    {
      motor_ref->last_cmd_ack = false;
    }
    break;
  }
}

void copyMotorBoardMessage(jaguar4x4_comms_msgs::msg::MotorBoard* to,
                           jaguar4x4_comms_msgs::msg::MotorBoard* from)
{
  to->amp_1 = from->amp_1;
  to->amp_2 = from->amp_2;
  to->motor_temp_1 = from->motor_temp_1;
  to->motor_temp_2 = from->motor_temp_2;
  to->encoder_pos_1 = from->encoder_pos_1;
  to->encoder_pos_2 = from->encoder_pos_2;
  to->encoder_diff_1 = from->encoder_diff_1;
  to->encoder_diff_2 = from->encoder_diff_2;
  to->digital_input = from->digital_input;
  to->digital_output = from->digital_output;
  to->motor_power_1 = from->motor_power_1;
  to->motor_power_2 = from->motor_power_2;
  to->encoder_vel_1 = from->encoder_vel_1;
  to->encoder_vel_2 = from->encoder_vel_2;
  to->board_temp_1 = from->board_temp_1;
  to->board_temp_2 = from->board_temp_2;
  to->volt_main = from->volt_main;
  to->volt_12v = from->volt_12v;
  to->volt_5v = from->volt_5v;
  to->motor_control_mode_1 = from->motor_control_mode_1;
  to->motor_control_mode_2 = from->motor_control_mode_2;
  to->overheat = from->overheat;
  to->overvoltage = from->overvoltage;
  to->undervoltage = from->undervoltage;
  to->e_short = from->e_short;
  to->estop = from->estop;
  to->last_cmd_ack = from->last_cmd_ack;
}
