#ifndef LingKongMotor_H_
#define LingKongMotor_H_

#include "CanBus.h"

#define LINGKONG_MOTOR_BASE_IDENTIFIER 0x140

class LingKongMotor : public CanDevice
{
public:
    enum class MotorType : uint8_t {
        MS = 0x01,
        MF = 0x02,
        MH = 0x03,
        MG = 0x04,
    };

    enum class MotorErrorFlag : uint8_t
    {
        None = 0x00,
        UnderVoltage = 0x01,
        OverTemperature = 0x08
    };


    /// @param rCanBus: reference to the CanBus object
    /// @param motorId: 1 to 32 (dip switch setting +1; e.g. dip switch set to 0 provides ID 1)
    LingKongMotor(CanBus &rCanBus, MotorType motorType, uint8_t motorId) : CanDevice(rCanBus, "LingKongMotor"), mMotorId(motorId), mMotorType(motorType) {};

    uint32_t GetMsgIdentifier() { return mMotorId + LINGKONG_MOTOR_BASE_IDENTIFIER; }
    uint8_t GetMotorId() { return mMotorId; }
    static uint8_t GetMotorIdFromMessageIdentifier(uint32_t msgIdentifier) { return msgIdentifier - LINGKONG_MOTOR_BASE_IDENTIFIER; }

    // ProcessMessage parses the message and invokes the appropriate Event method
    // override the virtual Event methods for your own use
    esp_err_t ProcessMessage(const twai_message_t &rMessage);


    enum class MotorCommand : uint8_t
    {
        MotorOff = 0x80, 
        MotorOn = 0x88,  
        MotorStop = 0x81,
        OpenLoopControl = 0xA0,
        TorqueClosedLoopControl = 0xA1,
        SpeedClosedLoopControl = 0xA2,
        MultiLoopAngleControl1 = 0xA3,
        MultiLoopAngleControl2 = 0xA4,
        SingleLoopAngleControl1 = 0xA5,
        SingleLoopAngleControl2 = 0xA6,
        IncrementAngleControl1 = 0xA7,
        IncrementAngleControl2 = 0xA8,
        ReadPIDParameter = 0x30,
        WritePIDParametersToRAM = 0x31,
        WritePIDParametersToROM = 0x32,
        ReadAcceleration = 0x33,
        WriteAccelerationToRAM = 0x34,
        ReadEncoder = 0x90,
        WriteEncoderValueToROM = 0x91,
        WriteCurrentPositionToROM = 0x19,
        ReadMultiAngleLoop = 0x92,
        ReadSingleAngleLoop = 0x94,
        ClearMotorAngleLoop = 0x95,
        ReadMotorState1AndErrorState = 0x9A,
        ClearMotorErrorState = 0x9B,
        ReadMotorState2 = 0x9C,
        ReadMotorState3 = 0x9D
    };

    virtual ~LingKongMotor()
    {
        // Destructor implementation goes here
    }

    esp_err_t Init();

    // single motor commands
    // Up to 32 (depending on the bus load) can be mounted on the same bus, in order to prevent bus conflicts, each driver needs to set a different ID, ID number 1~32. The master sends a single-motor command frame to the bus, and the corresponding ID motor executes
    // after receiving the command, and sends a reply frame with the same ID to the master after a period of time
    // (within 0.25ms). The command frame and reply frame message format are as follows:
    // Identifier:0x140 + ID(1~32)
    // Frame format: data frame
    // Frame type: standard frame
    // DLC:8byte
    // --------------------------------

    // Motor off command
    // Switch the motor from the on state (the default state after power-on) to the off state, clear motor
    // turns and the earlier command. The LED changes from always on to slow flashing. The motor can still
    // reply to commands, but will not perform actions.
    // Data field Instruction Data
    // DATA[0] Command byte 0x80
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    esp_err_t MotorOff();

    // Motor off event response is same as sending command
    virtual void EventMotorOff(){};

    // Motor on command
    // Swift the motor from off state to on state, LED changes from slow flashing to always on.
    // We can send commands to control the motor now.
    // Data field Instruction Data
    // DATA[0] Command byte 0x88
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    esp_err_t MotorOn();

    /// @brief Same as MotorOn, but explicitly reads the can bus as long the motor On is confirmed (up to timeout)
    ///        other CAN messages will be swallowed!! so use it only on startup
    /// @return 
    esp_err_t MotorOnConfirmed(unsigned int timeoutMs = 500);

    // Motor on event response is same as sending command
    virtual void EventMotorOn(){};

    // Motor stop command
    // Stop motor but don’t clear the motor state.Send commands again can control the motor
    esp_err_t MotorStop();

    // Motor stop event response is same as sending command
    virtual void EventMotorStop(){};

    // Open loop control command( The command can only be applied to MS series motor)
    // Host send commands to control the open loop voltage. PowerControl value is int16_t, range is -850~
    // 850,(Motor current and torque is different depends on motor
    //     Data field Instruction Data
    // DATA[0] Command byte 0xA0
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] PowerControl low byte DATA[4] = *(uint8_t *)(&powerControl)
    // DATA[5] PowerControl high byte DATA[5] = *((uint8_t *)(&powerControl)+1)
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    /// @param powerControl: Motor output power -850~850
    esp_err_t OpenLoopControl(int16_t powerControl);

    // Motor reply to the host after receiving the commands. The frame data include the following
    // parameters:
    // 1. Motor temperature,int8_t,1℃/LSB.
    // 2. Motor output power,int16_t, range is -850~850.
    // 3. Motor speed,int16_t,1dps/LSB.
    // 4. Encoder position,uint16_t, 15bit encoder range is 0~32767;18bit encoder range is
    //    0~65535(keeping high 16bit, Omit the lower 2 bit)
    //     Data field Instruction Data
    // DATA[0] Command byte 0xA0
    // DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
    // DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&power)
    // DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&power)+1)
    // DATA[4] Speed low byte DATA[4] = *(uint8_t *)(&speed)
    // DATA[5] Speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
    // DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
    // DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
    /// @param temperature: Motor temperature
    /// @param power: Motor output power -850~850
    /// @param speed: Motor speed in degrees per second
    /// @param encoder: Encoder position
    /// @param command: Command that triggered that response
    virtual void EventPowerMotorControl(int8_t temperature, int16_t power, int16_t speed, uint16_t encoder, uint8_t command){};

    // Torque closed loop control command(the command can only be applied to MF,MH,MG series motor)
    // Host send commands to control the torque current output, iqControl value is int16_t, range is -2048~
    // 2048, corresponding MF motor actual torque current range is -16.5A~16.5A, corresponding MG motor
    // actual torque current range is -33A~33A. The bus current and the actual torque of the motor vary from
    // motor to motor.
    // Iq (Quadrature Current): Represents the torque-producing current that generates the motor’s mechanical output.
    // Remark:
    // 1. In this command, iqControl is not limited by Max Torque Current of LK motor tool. Driver respond
    // Motor reply to the host after receiving the commands.
    /// @param iqControl: Motor torque current value -2048~2048
    esp_err_t TorqueClosedLoopControl(int16_t iqControl);

    // Event triggered by:
    // OpenLoopControl(), TorqueClosedLoopControl(), SpeedClosedLoopControl(), MultiLoopAngleControl1(), MultiLoopAngleControl2(), 
    // SingleLoopAngleControl1(), SingleLoopAngleControl2(), IncrementAngleControl1(), IncrementAngleControl2(),
    // ReadState2()
    // 1. Motor temperature,int8_t,1℃/LSB. 
    // 2. Motor torque current value iq,int16_t, range is -2048..2048,corresponding MF motor actual
    // torque current range is -16.5A..16.5A, corresponding MG motor actual torque current range
    // is-33A..33A. 
    // 3. Motor speed,int16_t,1dps/LSB. 
    // 4. Encoder position,uint16_t,14bit encoder range is 0..16383;15bit encoder range is 0~32767;18bit
    // encoder range is 0..65535(keeping high 16bit, Omit the lower 2 bit).
    // Iq (Quadrature Current): Represents the torque-producing current that generates the motor’s mechanical output.
    /// @param temperature: Motor temperature
    /// @param iq: Motor torque current value -2048~2048
    /// @param speed: Motor speed in degrees per second
    /// @param encoder: Encoder position
    /// @param command: Command that triggered that response
    virtual void EventIqMotorControl(int8_t temperature, int16_t iq, int16_t speed, uint16_t encoder, uint8_t command){};

    // Speed closed loop control command
    // Host send commands to control motor speed. SpeedControl value is int32_t, corresponding actual
    // speed is 0.01dps/LS
    // Remark:
    // 1. In this command, speed control is limited by Max speed value in LK motor tool.
    // 2. In this control mode, max acceleration is limited by Max Acceleration in LK motor tool.
    // 3. In this control mode,max torque current of MF/MH/MG motor is limited by Max Torque
    //    Current in LK motor tool. Max power of MS motor is limited by Max Power in LK motor
    //    tool
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA2). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA2)
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param speedControl: Motor speed in degrees per second
    esp_err_t SpeedClosedLoopControl(int32_t speedControl);


    // Multi loop angle control command 1(1 frame)
    // Host send this command to control the position of the motor(Multi turn angle). anglecontrol is
    // int32_t, corresponding actual position is 0.01degree/LSB, i.e 36000 corresponding to 360°,motor spin
    // direction is determined by the difference between the target position and the current position.
    // Data field Instruction Data
    // DATA[0] Command byte 0xA3
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // 9
    // DATA[4] Angle control low byte DATA[4] = *(uint8_t *)(&angleControl)
    // DATA[5] Angle control DATA[5] = *((uint8_t *)(&angleControl)+1)
    // DATA[6] Angle control DATA[6] = *((uint8_t *)(&angleControl)+2)
    // DATA[7] Angle control high byte DATA[7] = *((uint8_t *)(&angleControl)+3)
    // Remark:
    // 1. In this command, angle control is limited by Max Angle value in LK motor tool. 2. In this command, max speed is limited by Max Speed in LK motor tool. 3. In this control mode, max acceleration is limited by Max Acceleration in LK motor tool. 4. In this control mode,max torque current of MF/MH/MG motor is limited by Max Torque
    // Current in LK motor tool. Max power of MS motor is limited by Max Power in LK motor
    // tool.
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA3). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA3)
    // Iq (Quadrature Current): Represents the torque-producing current that generates the motor’s mechanical output.
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param angleControl: Motor angle control value in 0.01 degrees, i.e. 36000 for 360 degrees
    esp_err_t MultiLoopAngleControl1(int32_t angleControl);


    // Multi loop angle control command 2
    // Host send this command to control the position of the motor(Multi turn angle). 1. angleControl is int32_t,corresponding actual position is 0.01degree/LSB, i.e 36000
    // corresponding to 360°, motor spin direction is determined by the difference between the target
    // position and the current position. 2. maxSpeed limit the max speed, it is uint16_t, corresponding actual speed is 1dps/LSB, i.e 360
    // corresponding to 360dps. 
    // 1. In this command, angleControl value is limited by Max Angle of LK motor tool. 
    // 2. In this control mode, max acceleration of motor is limited by Max acceleration of LK
    // motor tool. 
    // 3. In this control mode, max torque current of MF/MH/MG motor is limited by Max Torque
    // Current of LK motor tool.Max power of MS motor is limited by Max Power of LK motor
    // tool. 
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA4). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA4)
    // Depending on MotorType, the event will be either 
    /// - EventPowerMotorControl() or 
    /// - EventIqMotorControl()
    /// @param angleControl: Motor angle control value in 0.01 degrees, i.e. 36000 for 360 degrees
    /// @param maxSpeed: Motor speed in degrees per second
    esp_err_t MultiLoopAngleControl2(int32_t angleControl, uint16_t maxSpeed);

    //     Single loop angle control 1(1 frame)
    // Host send this command to control the position of the motor(Single turn angle)
    // 1. spinDirection for motor spin direction setting, is uint8_t, 0x00 means clockwise,0x01 means
    // counterclockwise. 2. angleControl is uint32_t, corresponding actual position is 0.01degree/LSB, i.e 36000 means
    // 360°. 
    // 1. In this command, max speed of motor is limited by the Max speed of LK motor tool. 
    // 2. In this control mode, max acceleration of motor is limited by Max Acceleration of LK
    // motor tool. 
    // 3. In this control mode, max torque current of MF/MH/MG motor is limited by Max Torque
    // Current of LK motor tool.Max power of MS motor is limited by Max Power of LK motor
    // tool. Driver respond(1 frame)
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA5). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, its 0xA5)
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param spinDirection: Motor spin direction setting, 0x00 means clockwise, 0x01 means counterclockwise
    /// @param angleControl: Motor angle control value in 0.01 degrees, i.e. 36000 for 360 degrees
    esp_err_t SingleLoopAngleControl1(uint8_t spinDirection, uint32_t angleControl);


    // Single loop angle control 2(1 frame)
    // Host send this command to control the position of the motor(single turn angle). 1. .spinDirection for motor spin direction setting, is uint8_t, 0x00 means clockwise,0x01 means
    // counterclockwise
    // 2. angleControl is uint32_t, corresponding actual position is 0.01degree/LSB, i.e 36000 means
    // 360°. 3. maxSpeed is uint16_t,corresponding actual speed is 1dps/LSB,i.e 360 means 360dps. 
    // 1. In this control mode, max acceleration of motor is limited by Max Acceleration of LK
    // motor tool. 
    // 2. In this control mode, max torque current of MF/MH/MG motor is limited by Max Torque
    // Current of LK motor tool.Max power of MS motor is limited by Max Power of LK motor tool. Driver respond(1 frame)
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA6). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA6)
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param spinDirection: Motor spin direction setting, 0x00 means clockwise, 0x01 means counterclockwise
    /// @param angleControl: Motor angle control value in 0.01 degrees, i.e 36000 for 360 degrees
    /// @param maxSpeed: Motor speed in degrees per second
    esp_err_t SingleLoopAngleControl2(uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed);


    // Increment angle control command1(1 frame)
    // Host send commands to control the increment angle of the motor. angleIncrement is int32_t, corresponding actual position is 0.01degree/LSB, i.e 36000
    // corresponding to 360°,motor spin direction is determined by the symbol of parameter. Data field Instruction Data
    // DATA[0] Command byte 0xA7
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] Angle control low byte DATA[4] = *(uint8_t *)(& angleIncrement)
    // DATA[5] Angle control DATA[5] = *((uint8_t *)(& angleIncrement)+1)
    // DATA[6] Angle control DATA[6] = *((uint8_t *)(& angleIncrement)+2)
    // DATA[7] Angle control high byte DATA[7] = *((uint8_t *)(& angleIncrement)+3)
    // Remark:
    // 1. In this command, max speed of motor is limited by the Max speed of LK motor tool. 2. In this control mode, max acceleration of motor is limited by Max Acceleration of LK
    // motor tool. 3. In this control mode, max torque current of MF/MH/MG motor is limited by Max Torque
    // Current of LK motor tool.Max power of MS motor is limited by Max Power of LK motor tool. Driver respond(1 frame)
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA7). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA7).
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param angleIncrement: Motor angle increment value in 0.01 degrees, i.e. 36000 for 360 degrees
    esp_err_t IncrementAngleControl1(int32_t angleIncrement);


    // . Increment angle control command 2(1 frame)
    // Host send commands to control the increment angle of the motor. 1. angleIncrement is int32_t, corresponding actual position is 0.01degree/LSB, i.e 36000
    // corresponding to 360°,motor spin direction is determined by the symbol of parameter.
    // 12
    // 2. maxSpeed is uint32_t ,corresponding actual speed is 1dps/LSB, i.e360 means 360dps. Data field Instruction Data
    // DATA[0] Command byte 0xA8
    // DATA[1] NULL 0x00
    // DATA[2] Speed limit low byte DATA[2] = *(uint8_t *)(&maxSpeed)
    // DATA[3] Speed limit high byte DATA[3] = *((uint8_t *)(&maxSpeed)+1)
    // DATA[4] Angle control low byte DATA[4] = *(uint8_t *)(& angleIncrement)
    // DATA[5] Angle control DATA[5] = *((uint8_t *)(& angleIncrement)+1)
    // DATA[6] Angle control DATA[6] = *((uint8_t *)(& angleIncrement)+2)
    // DATA[7] Angle control high
    // byte
    // DATA[7] = *((uint8_t *)(& angleIncrement)+3)
    // Remark:
    // 1. In this control mode, max acceleration of motor is limited by Max Acceleration of LK
    // motor tool. 2. In this control mode, max torque current of MF/MH/MG motor is limited by Max Torque
    // Current of LK motor tool.Max power of MS motor is limited by Max Power of LK motor tool. Driver respond(1 frame)
    // Motor respond after receiving the host command.MS motor respond data is same as open loop
    // control command(only command byte is different, it’s 0xA8). MF/MH/MG motor respond data is same
    // as torque closed loop control command(only command byte is different, it’s 0xA8).
    // Depending on MotorType, the event will be either EventPowerMotorControl or EventIqMotorControl
    /// @param angleIncrement: Motor angle increment value in 0.01 degrees, i.e 36000 for 360 degrees
    esp_err_t IncrementAngleControl2(int32_t angleIncrement, uint32_t maxSpeed);


    // . Read PID parameter command(1 frame)
    // Host send this command to read current motor PID parameters. Data field Instruction Data
    // DATA[0] Command byte 0x30
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    esp_err_t ReadPIDParameter();

    // Driver respond data includes PI parameters of position/speed/toque loop. Data field Instruction Data
    // DATA[0] Command byte 0x30
    // DATA[1] NULL 0x00
    // DATA[2] Position loop P parameter DATA[2] = anglePidKp
    // DATA[3] Position loop I parameter DATA[3] = anglePidKi
    // DATA[4] Speed loop P parameter DATA[4] = speedPidKp
    // DATA[5] Speed loop I parameter DATA[5] = speedPidKi
    // DATA[6] Toque loop P parameter DATA[6] = iqPidKp
    // DATA[7] Toque loop I parameter DATA[7] = iqPidKi
    /// @param anglePidKp: Position loop P parameter
    /// @param anglePidKi: Position loop I parameter
    /// @param speedPidKp: Speed loop P parameter
    /// @param speedPidKi: Speed loop I parameter
    /// @param iqPidKp: Toque loop P parameter
    /// @param iqPidKi: Toque loop I parameter
    virtual void EventReadPIDParameter(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi){};

    // Write PID parameters to RAM command (1 frame)
    // Host send this command to write PID parameters to RAM, parameter are invalid when power off. DATA[0] Command byte 0x31
    // DATA[1] NULL 0x00
    // Data field
    // Instruction Data
    // DATA[2] Position loop P parameter DATA[2] = anglePidKp
    // DATA[3] Position loop I parameter DATA[3] = anglePidKi
    // DATA[4] Speed loop P parameter DATA[4] = speedPidKp
    // DATA[5] Speed loop I parameter DATA[5] = speedPidKi
    // DATA[6] Toque loop P parameter DATA[6] = iqPidKp
    // DATA[7] Toque loop I parameter DATA[7] = iqPidKi
    /// @param anglePidKp: Position loop P parameter
    /// @param anglePidKi: Position loop I parameter
    /// @param speedPidKp: Speed loop P parameter
    /// @param speedPidKi: Speed loop I parameter
    /// @param iqPidKp: Toque loop P parameter
    /// @param iqPidKi: Toque loop I parameter
    esp_err_t WritePIDParametersToRAM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);


    // Driver response after writing PID parameters to RAM or ROM
    /// @param anglePidKp: Position loop P parameter
    /// @param anglePidKi: Position loop I parameter
    /// @param speedPidKp: Speed loop P parameter
    /// @param speedPidKi: Speed loop I parameter
    /// @param iqPidKp: Toque loop P parameter
    /// @param iqPidKi: Toque loop I parameter
    virtual void EventPIDParameters(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi, uint8_t command){};

    // Write PID parameters to ROM command(1 frame)
    // Host sent this command to write PID paramters to ROM, parameters are valid when power off. Data field Instruction Data
    // DATA[0] Command byte 0x32
    // DATA[1] NULL 0x00
    // DATA[2] Position loop P parameter DATA[2] = anglePidKp
    // DATA[3] Position loop I parameter DATA[3] = anglePidKi
    // DATA[4] Speed loop P parameter DATA[4] = speedPidKp
    // DATA[5] Speed loop I parameter DATA[5] = speedPidKi
    // DATA[6] Toque loop P parameter DATA[6] = iqPidKp
    // DATA[7] Toque loop I parameter DATA[7] = iqPidKi
    // responds with EventPIDParameters event
    esp_err_t WritePIDParametersToROM(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);


    // Read acceleration command(1 frame)
    // Host sent this command to read motor acceleration parameter. Data field Instruction Data
    // DATA[0] Command byte 0x33
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // Driver respond(1 frame)
    esp_err_t ReadAcceleration();

    // Driver respond data includes acceleration data. DataAccel is int32_t ,unit is 1dps/s
    // Data field Instruction Data
    // DATA[0] Command byte 0x33
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] Acceleration low byte1 DATA[4] = *(uint8_t *)(&Accel)
    // DATA[5] Acceleration byte 2 DATA[5] = *((uint8_t *)(&Accel)+1)
    // DATA[6] Acceleration byte 3 DATA[6] = *((uint8_t *)(&Accel)+2)
    // DATA[7] Acceleration byte 4 DATA[7] = *((uint8_t *)(&Accel)+3)
    /// @param accel: Motor acceleration in degrees per second per second
    virtual void EventAcceleration(int32_t accel, uint8_t command){};

    // Write acceleration to RAM command(1 frame)
    // Host send commands to write acceleration to RAM.Parameter are invalid when power off. DataAccel is
    // int32_t,unit is 1dps/s
    // Data field Instruction Data
    // DATA[0] Command byte 0x34
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] Acceleration low byte1 DATA[4] = *(uint8_t *)(&Accel)
    // DATA[5] Acceleration byte 2 DATA[5] = *((uint8_t *)(&Accel)+1)
    // DATA[6] Acceleration byte 3 DATA[6] = *((uint8_t *)(&Accel)+2)
    // DATA[7] Acceleration byte 4 DATA[7] = *((uint8_t *)(&Accel)+3)
    // Driver respond(1 frame)
    // Driver respond after receiving the command. Responding command is same as receiving.
    // response with EventAcceleration event
    /// @param accel: Motor acceleration in degrees per second per second
    esp_err_t WriteAccelerationToRAM(int32_t accel);


    // Read encoder command (1 frame)
    // Host send commands to read current encoder position. Data field Instruction Data
    // DATA[0] Command byte 0x90
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    esp_err_t ReadEncoder();

    // Motor reply to the host after receiving the commands. The frame data include the following
    // parameters:
    // 1. encoder(uint16_t, 14 bit encoder range is 0~16383), is encoder raw value minus encoder offset
    // value.
    // 15
    // 2. encoderRaw(uint16_t,14 bit encoder range is 0~16383). 3. encoderOffset(uint16_t,14 bit encoder range is 0~16383). This point is the initial zero position
    // of the motor. Data field Instruction Data
    // DATA[0] Command byte 0x90
    // DATA[1] NULL 0x00
    // DATA[2] Encoder low byte DATA[2] = *(uint8_t *)(&encoder)
    // DATA[3] Encoder high byte DATA[3] = *((uint8_t *)(&encoder)+1)
    // DATA[4] Encoder raw position low
    // byte
    // DATA[4] = *(uint8_t *)(&encoderRaw)
    // DATA[5] Encoder raw position high
    // byte
    // DATA[5] = *((uint8_t *)(&encoderRaw)+1)
    // DATA[6] Encoder offset low byte DATA[6] = *(uint8_t *)(&encoderOffset)
    // DATA[7] Encoder offset high byte DATA[7] = *((uint8_t *)(&encoderOffset)+1)
    /// @param encoder: Encoder position
    /// @param encoderRaw: Encoder raw position
    /// @param encoderOffset: Encoder offset
    virtual void EventReadEncoder(uint16_t encoder, uint16_t encoderRaw, uint16_t encoderOffset){};

    // Write encoder value to ROM as the motor zero point command(1 frame)
    // Host send this command to set encoder offset. EncoderOffset is uint16_t,14bit encoder range is
    // 0~16383. Data field Instruction Data
    // DATA[0] Command byte 0x91
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] Encoder offset low byte DATA[6] = *(uint8_t *)(&encoderOffset)
    // DATA[7] Encoder offset high byte DATA[7] = *((uint8_t *)(&encoderOffset)+1)
    // Driver respond(1 frame)
    // Driver respond after receiving the command. Responding command is same as receiving.
    /// @param encoderOffset: Encoder offset
    esp_err_t WriteEncoderValueToROM(uint16_t encoderOffset);

    // Driver respond after receiving the command. Responding command is same as receiving.
    /// @param encoderOffset: Encoder offset
    virtual void EventEncoderOffset(uint16_t encoderOffset, uint8_t command){};

    // Write current position to ROM as the motor zero point command(1 frame)
    // Write motor encoder current position to ROM as the initial position. Remark:
    // 1． The command will be valid only after reset power. 2． This command will write the zero point to the driver's ROM, multiple writes will affect the
    // chip life, and frequent use is not recommended. Data field Instruction Data
    // DATA[0] Command byte 0x19
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // 16
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // responds with EventEncoderOffset event
    esp_err_t WriteCurrentPositionToROM();

    // Read multi angle loop command(1 frame)
    // Host send this command to read current motor multi angle absolute value. Data field Instruction Data
    // DATA[0] Command byte 0x92
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // Driver respond(1 frame)
    // Motor reply to the host after receiving the commands. The frame data include the following
    // parameters:
    esp_err_t ReadMultiAngleLoop();

    // motorAngle is int64_t, positive values represent clockwise cumulative angles, and negative
    // values represent counterclockwise cumulative angles, unit 0.01°/LSB. Data field Instruction Data
    /// @param motorAngle: Motor angle in 0.01 degrees
    virtual void EventMultiAngleLoop(int64_t motorAngle){};

    // Read single angle loop command(1 frame)
    // Host send this command to read motor current single angle value.
    // 17
    // Data field Instruction Data
    // DATA[0] Command byte 0x94
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // Driver respond(1 frame)
    // Motor reply to the host after receiving the commands. The frame data include the following
    esp_err_t ReadSingleAngleLoop();

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
    /// @param circleAngle: Motor single angle in 0.01 degrees
    virtual void EventSingleAngleLoop(uint32_t circleAngle){};

    // Clear motor angle loop command(1 frame)Not yet unavailable
    // This command clear motor multi turn and single turn data and set current position as motor zero
    // point. It’s invalid when power off. Remark: This command will clear all the position loop command at the same time. Data field Instruction Data
    // DATA[0] Command byte 0x95
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // Driver respond(1 frame)
    // Driver respond after receiving the host command.Frame data is same as the host sending command.
    esp_err_t ClearMotorAngleLoop();

    // Motor respond after receiving the host command.Motor respond data is same as the host sending
    // command.
    virtual void EventClearMotorAngleLoop(){};

    // Read motor state 1 and error state command(1 frame)
    // This command read current motor temperature, voltage and error state.
    // Triggers an EventMotorAndErrorState() event
    esp_err_t ReadMotorState1AndErrorState();

    // 1. Motor temperature(int8_t, 1℃/LSB). 2. Motor voltage(uint16_t, 0.1V/LSB). 3. errorState(uint8_t, the bits represent different motor states)
    /// Triggered by:
    /// - ReadMotorState1AndErrorState()
    /// @param temperature: Motor temperature in degrees Celsius
    /// @param voltage: Motor voltage in 0.1 volts
    /// @param errorState: Motor error state
    ///                     bit 0: Voltage state 0: Voltage is normal 1: Under Voltage protect
    ///                     bit 3: Temperature state Temperature is 0: normal 1: Over temperature protect
    virtual void EventMotorAndErrorState(int8_t temperature, uint16_t voltage, uint8_t flagsErrorState, uint8_t command){};

    /// Clear motor error state and trigger EventMotorAndErrorState event
    /// This command is to clear motor current error state. Data field Instruction Data
    esp_err_t ClearMotorErrorState();

    /// Read motor state and trigger void <see cref="LingKongMotor::EventIqMotorControl()"/> event
    /// This command read current temperature,iq,speed and encoder position. 
    esp_err_t ReadMotorState2();

    // Read motor state 3 command (1 frame)
    // This command read current motor temperature and phase current data. Data field Instruction Data
    // DATA[0] Command byte 0x9D
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    esp_err_t ReadMotorState3();

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
    virtual void EventReadMotorState3(int8_t temperature, int16_t iA, int16_t iB, int16_t iC){};

private:
    // initialize message with default values
    inline void InitializeMessage(twai_message_t &rMessage);
    void ParseMotorControlMessage(const twai_message_t &rMessage);
    void ParsePIDParameters(const twai_message_t &rMessage);
    void ParseAcceleration(const twai_message_t &rMessage);
    void ParseMotorAndErrorState(const twai_message_t &rMessage);
    uint8_t mMotorId;
    const MotorType mMotorType;
};


#endif