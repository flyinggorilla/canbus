#include "LingKongMotor.h"
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_log.h>
#include <esp_task.h>
#include <esp_timer.h>

static const char *tag = "LingKongMotor";

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

esp_err_t LingKongMotor::Init()
{
  esp_log_level_set(tag, LOG_LOCAL_LEVEL);
  return MotorOff();
}

void LingKongMotor::InitializeMessage(twai_message_t &rMessage)
{
  rMessage = {};
  rMessage.identifier = GetMsgIdentifier();
  rMessage.flags = TWAI_MSG_FLAG_NONE;
  rMessage.data_length_code = 8;
}

esp_err_t LingKongMotor::MotorOff()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = static_cast<uint8_t>(MotorCommand::MotorOff);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::MotorOn()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::MotorOn;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::MotorOnConfirmed(unsigned int timeoutMs)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::MotorOn;
  esp_err_t ret = mrCanBus.TransmitMessage(&message);
  if (ret != ESP_OK) return ret;

  int64_t startTime = esp_timer_get_time();
  while (esp_timer_get_time() - startTime < timeoutMs * 1000)
  {
    ret = mrCanBus.ReceiveMessage(&message);
    if (ret != ESP_OK) return ret;

    if (message.identifier == GetMsgIdentifier() && message.data[0] == (uint8_t)MotorCommand::MotorOn)
    {
      return ESP_OK;
    }
  }

  return ESP_ERR_TIMEOUT;
}

esp_err_t LingKongMotor::MotorStop()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::MotorStop;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::OpenLoopControl(int16_t powerControl)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::OpenLoopControl;
  message.data[4] = *(uint8_t *)(&powerControl);
  message.data[5] = *((uint8_t *)(&powerControl) + 1);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::TorqueClosedLoopControl(int16_t iqControl)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::TorqueClosedLoopControl;
  message.data[4] = *(uint8_t *)(&iqControl);
  message.data[5] = *((uint8_t *)(&iqControl) + 1);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::SpeedClosedLoopControl(int32_t speedControl)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::SpeedClosedLoopControl;
  message.data[4] = *(uint8_t *)(&speedControl);
  message.data[5] = *((uint8_t *)(&speedControl) + 1);
  message.data[6] = *((uint8_t *)(&speedControl) + 2);
  message.data[7] = *((uint8_t *)(&speedControl) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::MultiLoopAngleControl1(int32_t angleControl)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::MultiLoopAngleControl1;
  message.data[4] = *(uint8_t *)(&angleControl);
  message.data[5] = *((uint8_t *)(&angleControl) + 1);
  message.data[6] = *((uint8_t *)(&angleControl) + 2);
  message.data[7] = *((uint8_t *)(&angleControl) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::MultiLoopAngleControl2(int32_t angleControl, uint16_t maxSpeed)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::MultiLoopAngleControl2;
  message.data[2] = *(uint8_t *)(&maxSpeed);
  message.data[3] = *((uint8_t *)(&maxSpeed) + 1);
  message.data[4] = *(uint8_t *)(&angleControl);
  message.data[5] = *((uint8_t *)(&angleControl) + 1);
  message.data[6] = *((uint8_t *)(&angleControl) + 2);
  message.data[7] = *((uint8_t *)(&angleControl) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::SingleLoopAngleControl1(uint8_t spinDirection, uint32_t angleControl)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::SingleLoopAngleControl1;
  message.data[1] = spinDirection;
  message.data[2] = *(uint8_t *)(&angleControl);
  message.data[3] = *((uint8_t *)(&angleControl) + 1);
  message.data[4] = *((uint8_t *)(&angleControl) + 2);
  message.data[5] = *((uint8_t *)(&angleControl) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::SingleLoopAngleControl2(uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::SingleLoopAngleControl2;
  message.data[1] = spinDirection;
  message.data[2] = *(uint8_t *)(&maxSpeed);
  message.data[3] = *((uint8_t *)(&maxSpeed) + 1);
  message.data[4] = *(uint8_t *)(&angleControl);
  message.data[5] = *((uint8_t *)(&angleControl) + 1);
  message.data[6] = *((uint8_t *)(&angleControl) + 2);
  message.data[7] = *((uint8_t *)(&angleControl) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::IncrementAngleControl1(int32_t angleIncrement)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::IncrementAngleControl1;
  message.data[4] = *(uint8_t *)(&angleIncrement);
  message.data[5] = *((uint8_t *)(&angleIncrement) + 1);
  message.data[6] = *((uint8_t *)(&angleIncrement) + 2);
  message.data[7] = *((uint8_t *)(&angleIncrement) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::IncrementAngleControl2(int32_t angleIncrement, uint32_t maxSpeed)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::IncrementAngleControl2;
  message.data[2] = *(uint8_t *)(&maxSpeed);
  message.data[3] = *((uint8_t *)(&maxSpeed) + 1);
  message.data[4] = *(uint8_t *)(&angleIncrement);
  message.data[5] = *((uint8_t *)(&angleIncrement) + 1);
  message.data[6] = *((uint8_t *)(&angleIncrement) + 2);
  message.data[7] = *((uint8_t *)(&angleIncrement) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadPIDParameter()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = 0x30;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::WritePIDParametersToRAM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::WritePIDParametersToRAM;
  message.data[2] = anglePidKp;
  message.data[3] = anglePidKi;
  message.data[4] = speedPidKp;
  message.data[5] = speedPidKi;
  message.data[6] = iqPidKp;
  message.data[7] = iqPidKi;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::WritePIDParametersToROM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::WritePIDParametersToROM;
  message.data[2] = anglePidKp;
  message.data[3] = anglePidKi;
  message.data[4] = speedPidKp;
  message.data[5] = speedPidKi;
  message.data[6] = iqPidKp;
  message.data[7] = iqPidKi;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor:: ReadAcceleration()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadAcceleration;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::WriteAccelerationToRAM(int32_t accel)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::WriteAccelerationToRAM;
  message.data[4] = *(uint8_t *)(&accel);
  message.data[5] = *((uint8_t *)(&accel) + 1);
  message.data[6] = *((uint8_t *)(&accel) + 2);
  message.data[7] = *((uint8_t *)(&accel) + 3);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadEncoder()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadEncoder;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::WriteEncoderValueToROM(uint16_t encoderOffset)
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::WriteEncoderValueToROM;
  message.data[6] = *(uint8_t *)(&encoderOffset);
  message.data[7] = *((uint8_t *)(&encoderOffset) + 1);
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::WriteCurrentPositionToROM()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::WriteCurrentPositionToROM;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadMultiAngleLoop()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadMultiAngleLoop;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadSingleAngleLoop()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadSingleAngleLoop;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ClearMotorAngleLoop()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ClearMotorAngleLoop;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadMotorState1AndErrorState()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadMotorState1AndErrorState;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ClearMotorErrorState()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ClearMotorErrorState;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadMotorState2()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadMotorState2;
  return mrCanBus.TransmitMessage(&message);
}

esp_err_t LingKongMotor::ReadMotorState3()
{
  twai_message_t message;
  InitializeMessage(message);
  message.data[0] = (uint8_t)MotorCommand::ReadMotorState3;
  return mrCanBus.TransmitMessage(&message);
}

void LingKongMotor::ParseMotorControlMessage(const twai_message_t &rMessage)
{
  uint8_t command = rMessage.data[0];
  int8_t temperature = rMessage.data[1];
  int16_t speed = 0;
  speed |= static_cast<int16_t>(rMessage.data[4]);
  speed |= static_cast<int16_t>(rMessage.data[5]) << 8;
  uint16_t encoder = 0;
  encoder |= static_cast<uint16_t>(rMessage.data[6]);
  encoder |= static_cast<uint16_t>(rMessage.data[7]) << 8;
  if ((mMotorType == MotorType::MS) && (command != static_cast<uint8_t>(MotorCommand::ReadMotorState2)))
  {
      int16_t power = 0;
      power |= static_cast<int16_t>(rMessage.data[2]);
      power |= static_cast<int16_t>(rMessage.data[3]) << 8;
      ESP_LOGV(tag, "EventPOWER motor Control: temperature=%d, power=%d, speed=%d, encoder=%d, command=%02Xh", temperature, power, speed, encoder, command);
      EventPowerMotorControl(temperature, power, speed, encoder, command);
    }
    else
    {
      int16_t iq = 0;
      iq |= static_cast<int16_t>(rMessage.data[2]);
      iq |= static_cast<int16_t>(rMessage.data[3]) << 8;
      ESP_LOGV(tag, "EventIQ Motor Control: temperature=%d, iq=%d, speed=%d, encoder=%d, command=%02Xh", temperature, iq, speed, encoder, command); 
      EventIqMotorControl(temperature, iq, speed, encoder, command);
  }
}

void LingKongMotor::ParsePIDParameters(const twai_message_t &rMessage)
{
  uint8_t command = rMessage.data[0];
  uint8_t anglePidKp = rMessage.data[2];
  uint8_t anglePidKi = rMessage.data[3];
  uint8_t speedPidKp = rMessage.data[4];
  uint8_t speedPidKi = rMessage.data[5];
  uint8_t iqPidKp = rMessage.data[6];
  uint8_t iqPidKi = rMessage.data[7];
  EventPIDParameters(anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi, command);
}

void LingKongMotor::ParseAcceleration(const twai_message_t &rMessage)
{
  uint8_t command = rMessage.data[0];
  int32_t accel = 0;
  accel |= static_cast<int32_t>(rMessage.data[4]);
  accel |= static_cast<int32_t>(rMessage.data[5]) << 8;
  accel |= static_cast<int32_t>(rMessage.data[6]) << 16;
  accel |= static_cast<int32_t>(rMessage.data[7]) << 24;
  EventAcceleration(accel, command);
}

void LingKongMotor::ParseMotorAndErrorState(const twai_message_t &rMessage)
{
  //     Motor reply to the host after receiving the commands. The frame data include the following
  // parameters:
  // 1. Motor temperature(int8_t, 1℃/LSB). 2. Motor voltage(uint16_t, 0.1V/LSB). 3. errorState(uint8_t, the bits represent different motor states)
  // Data field Instruction Data
  // DATA[0] Command byte 0x9A
  // DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
  // DATA[2] Voltage low byte DATA[3] = *(uint8_t *)(&voltage)
  // DATA[3] Voltage high byte DATA[4] = *((uint8_t *)(& voltage)+1)
  // WRONG: DATA[2] NULL 0x00
  // WRONG: DATA[3] Voltage low byte DATA[3] = *(uint8_t *)(&voltage)
  // WRONG: DATA[4] Voltage high byte DATA[4] = *((uint8_t *)(& voltage)+1)
  // DATA[5] NULL 0x00
  // DATA[6] NULL 0x00
  // DATA[7] Error state byte DATA[7]=errorState
  // Remark:
  // errorState each bit specific state sheet:
  // errorState bit State Instruction 0 1
  // 0 Voltage state 0: Voltage is normal 1: Under Voltage protect
  // 1 invalid
  // 2 invalid
  // 3 Temperature state Temperature is 0: normal 1: Over temperature protect
  // 4 invalid
  // 5 invalid
  // 6 invalid
  // 7 invalid
  /// @param temperature: Motor temperature in degrees Celsius
  /// @param voltage: Motor voltage in 0.1 volts
  /// @param errorState: Motor error state
  ///                     bit 0: Voltage state 0: Voltage is normal 1: Under Voltage protect
  ///                     bit 3: Temperature state Temperature is 0: normal 1: Over temperature protect
  ESP_LOGV(tag, "Voltage Low Byte: 0x%02Xh, Voltage High Byte: 0x%02Xh", rMessage.data[3], rMessage.data[4]);

  uint8_t command = rMessage.data[0];
  int8_t temperature = rMessage.data[1];
  uint16_t voltage = 0;
  voltage |= static_cast<uint16_t>(rMessage.data[2]);  // NOTE: documentation is wrong as of v2.35, voltage is in data[2] and data[3]
  voltage |= static_cast<uint16_t>(rMessage.data[3]) << 8;
  ESP_LOGV(tag, "Voltage: %u, Command: 0x%02Xh", voltage, command);
  // log rMessage data 3 and data 4 to hex values
  uint8_t flagsErrorState = rMessage.data[7];
  EventMotorAndErrorState(temperature, voltage, flagsErrorState, command);
}

esp_err_t LingKongMotor::ProcessMessage(const twai_message_t &message)
{

  // create a command variable from rMessage
  // uint8_t command = rMessage.data[0];

  // create a switch list based on rMessage.data[0] which is the command byte
  switch (message.data[0])
  {
  case (uint8_t)MotorCommand::MotorOff:
    ESP_LOGV(tag, "Motor Off");
    EventMotorOff();
    break;
  case (uint8_t)MotorCommand::MotorOn:
    ESP_LOGV(tag, "Motor On");
    EventMotorOn();
    break;
  case (uint8_t)MotorCommand::MotorStop:
    ESP_LOGV(tag, "Motor Stop");
    EventMotorStop();
    break;
  case (uint8_t)MotorCommand::OpenLoopControl:
  {
    ESP_LOGV(tag, "Open Loop Control");
    ParseMotorControlMessage(message);
    break;
  }
  case (uint8_t)MotorCommand::TorqueClosedLoopControl:
  {
    ESP_LOGV(tag, "Torque Closed Loop Control");
    ParseMotorControlMessage(message);
    break;
  }
  case (uint8_t)MotorCommand::SpeedClosedLoopControl:
    ESP_LOGV(tag, "Speed Closed Loop Control");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::MultiLoopAngleControl1:
    ESP_LOGV(tag, "Multi Loop Angle Control 1");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::MultiLoopAngleControl2:
    ESP_LOGV(tag, "Multi Loop Angle Control 2");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::SingleLoopAngleControl1:
    ESP_LOGV(tag, "Single Loop Angle Control 1");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::SingleLoopAngleControl2:
    ESP_LOGV(tag, "Single Loop Angle Control 2");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::IncrementAngleControl1:
    ESP_LOGV(tag, "Increment Angle Control 1");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::IncrementAngleControl2:
    ESP_LOGV(tag, "Increment Angle Control 2");
    ParseMotorControlMessage(message);
    break;
  case (uint8_t)MotorCommand::ReadPIDParameter:
    ESP_LOGV(tag, "Read PID Parameter");
    // create a method to parse the PID parameters from the message similar to ParseMotorControlMessage
    ParsePIDParameters(message);
    break;
  case (uint8_t)MotorCommand::WritePIDParametersToRAM:
    ESP_LOGV(tag, "Write PID Parameters to RAM");
    ParsePIDParameters(message);
    break;
  case (uint8_t)MotorCommand::WritePIDParametersToROM:
    ESP_LOGV(tag, "Write PID Parameters to ROM");
    ParsePIDParameters(message);
    break;
  case (uint8_t)MotorCommand::ReadAcceleration:
    ESP_LOGV(tag, "Read Acceleration");
    ParseAcceleration(message);
    break;
  case (uint8_t)MotorCommand::WriteAccelerationToRAM:
    ESP_LOGV(tag, "Write Acceleration to RAM");
    ParseAcceleration(message);
    break;
  case (uint8_t)MotorCommand::ReadEncoder:
    ESP_LOGV(tag, "Read Encoder");
    break;
  case (uint8_t)MotorCommand::WriteEncoderValueToROM:
    ESP_LOGV(tag, "Write Encoder Value to ROM");
    break;
  case (uint8_t)MotorCommand::WriteCurrentPositionToROM:
    ESP_LOGV(tag, "Write Current Position to ROM");
    break;
  case (uint8_t)MotorCommand::ReadMultiAngleLoop:
  {

    ESP_LOGV(tag, "Read Multi Angle Loop");
    // Motor reply to the host after receiving the commands. The frame data include the following
    // parameters:
    // 1. motorAngle is int64_t, positive values represent clockwise cumulative angles, and negative
    // values represent counterclockwise cumulative angles, unit 0.01°/LSB. Data field Instruction Data
    // DATA[0] Command byte 0x92
    // DATA[1] Angle low byte1 DATA[1] = *(uint8_t *)(&motorAngle)
    // DATA[2] Angle byte2 DATA[2] = *((uint8_t *)(& motorAngle)+1)
    // DATA[3] Angle byte3 DATA[3] = *((uint8_t *)(& motorAngle)+2)
    // DATA[4] Angle byte4 DATA[4] = *((uint8_t *)(& motorAngle)+3)
    // DATA[5] Angle byte5 DATA[5] = *((uint8_t *)(& motorAngle)+4)
    // DATA[6] Angle byte6 DATA[6] = *((uint8_t *)(& motorAngle)+5)
    // DATA[7] Angle byte7 DATA[7] = *((uint8_t *)(& motorAngle)+6)
    int64_t motorAngle = 0;
    motorAngle |= static_cast<int64_t>(message.data[1]);
    motorAngle |= static_cast<int64_t>(message.data[2]) << 8;
    motorAngle |= static_cast<int64_t>(message.data[3]) << 16;
    motorAngle |= static_cast<int64_t>(message.data[4]) << 24;
    motorAngle |= static_cast<int64_t>(message.data[5]) << 32;
    motorAngle |= static_cast<int64_t>(message.data[6]) << 40;
    motorAngle |= static_cast<int64_t>(message.data[7]) << 48;
    // if the highest bit of message.data[7] is set, then the motorAngle is negative
    if (message.data[7] & 0x80)
    {
      motorAngle |= 0xFF00000000000000; // add the 8th byte for a valid 64bit negative number
    }
    EventMultiAngleLoop(motorAngle);
    // log the motorAngle as float to 2 decimal places
    ESP_LOGV(tag, "Motor Angle: %lli, %0.1f", motorAngle, (float)(motorAngle / 100.0));
    break;
  }
  case (uint8_t)MotorCommand::ReadSingleAngleLoop:
  {
    ESP_LOGV(tag, "Read Single Angle Loop");
    // 1. circleAngle is motor single loop, is uint32_t,Starting at the zero point of the encoder, it increases
    // clockwise, and the value returns to 0 when the zero point is reached again,unit is
    // 0.01°/LSB,value range is 0~36000-1. Data field Instruction Data
    // DATA[0] Command byte 0x94
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] Single angle low byte1 DATA[4] = *(uint8_t *)(& circleAngle)
    // DATA[5] Single angle byte2 DATA[5] = *((uint8_t *)(& circleAngle)+1)
    // DATA[6] Single angle byte3 DATA[6] = *((uint8_t *)(& circleAngle)+2)
    // DATA[7] Single angle high byte4 DATA[7] = *((uint8_t *)(& circleAngle)+3)
    uint32_t circleAngle = 0;
    circleAngle |= static_cast<uint32_t>(message.data[4]);
    circleAngle |= static_cast<uint32_t>(message.data[5]) << 8;
    circleAngle |= static_cast<uint32_t>(message.data[6]) << 16;
    circleAngle |= static_cast<uint32_t>(message.data[7]) << 24;
    EventSingleAngleLoop(circleAngle);
  }
  break;
  case (uint8_t)MotorCommand::ClearMotorAngleLoop:
    ESP_LOGV(tag, "Clear Motor Angle Loop");
    EventClearMotorAngleLoop();
    break;
  case (uint8_t)MotorCommand::ReadMotorState1AndErrorState:
    ESP_LOGV(tag, "Read Motor State 1 and Error State");
    ParseMotorAndErrorState(message);
    break;
  case (uint8_t)MotorCommand::ClearMotorErrorState:
    ESP_LOGV(tag, "Clear Motor Error State");
    ParseMotorAndErrorState(message);
    break;
  case (uint8_t)MotorCommand::ReadMotorState2:
  {
    ESP_LOGV(tag, "Read Motor State 2");
    ParseMotorControlMessage(message);
    break;
  }
  case (uint8_t)MotorCommand::ReadMotorState3:
  {
    ESP_LOGV(tag, "Read Motor State 3");
    // Motor reply to the host after receiving the commands. The frame data include the following
    // parameters:
    // 1. temperature(int8_t ,1℃/LSB)
    // 2. A phase current, int16_t,corresponding actual phase current is 1A/64LSB. 3. B phase current, int16_t,corresponding actual phase current is 1A/64LSB. 4. C phase current, int16_t,corresponding actual phase current is 1A/64LSB. Data field Instruction Data
    // DATA[0] Command byte 0x9D
    // DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
    // DATA[2] A phase current low byte DATA[2] = *(uint8_t *)(&iA)
    // DATA[3] A phase current high byte DATA[3] = *((uint8_t *)(& iA)+1)
    // DATA[4] B phase current low byte DATA[4] = *(uint8_t *)(&iB)
    // DATA[5] B phase current high byte DATA[5] = *((uint8_t *)(& iB)+1)
    // DATA[6] C phase current low byte DATA[6] = *(uint8_t *)(&iC)
    // DATA[7] C phase current high byte DATA[7] = *((uint8_t *)(& iC)+1)
    int8_t temperature = message.data[1];
    int16_t iA = 0;
    iA |= static_cast<int16_t>(message.data[2]);
    iA |= static_cast<int16_t>(message.data[3]) << 8;
    int16_t iB = 0;
    iB |= static_cast<int16_t>(message.data[4]);
    iB |= static_cast<int16_t>(message.data[5]) << 8;
    int16_t iC = 0;
    iC |= static_cast<int16_t>(message.data[6]);
    iC |= static_cast<int16_t>(message.data[7]) << 8;
    EventReadMotorState3(temperature, iA, iB, iC);
    break;
  }
  default:
    ESP_LOGV(tag, "Unknown command");
    return false;
  }

  return true;
}
