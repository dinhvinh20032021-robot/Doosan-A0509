// Auto-generated. Do not edit!

// (in-package dsr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotStateRT {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_stamp = null;
      this.actual_joint_position = null;
      this.actual_joint_position_abs = null;
      this.actual_joint_velocity = null;
      this.actual_joint_velocity_abs = null;
      this.actual_tcp_position = null;
      this.actual_tcp_velocity = null;
      this.actual_flange_position = null;
      this.actual_flange_velocity = null;
      this.actual_motor_torque = null;
      this.actual_joint_torque = null;
      this.raw_joint_torque = null;
      this.raw_force_torque = null;
      this.external_joint_torque = null;
      this.external_tcp_force = null;
      this.target_joint_position = null;
      this.target_joint_velocity = null;
      this.target_joint_acceleration = null;
      this.target_motor_torque = null;
      this.target_tcp_position = null;
      this.target_tcp_velocity = null;
      this.jacobian_matrix = null;
      this.gravity_torque = null;
      this.coriolis_matrix = null;
      this.mass_matrix = null;
      this.solution_space = null;
      this.singularity = null;
      this.operation_speed_rate = null;
      this.joint_temperature = null;
      this.controller_digital_input = null;
      this.controller_digital_output = null;
      this.controller_analog_input_type = null;
      this.controller_analog_input = null;
      this.controller_analog_output_type = null;
      this.controller_analog_output = null;
      this.flange_digital_input = null;
      this.flange_digital_output = null;
      this.flange_analog_input = null;
      this.external_encoder_strobe_count = null;
      this.external_encoder_count = null;
      this.goal_joint_position = null;
      this.goal_tcp_position = null;
      this.robot_mode = null;
      this.robot_state = null;
      this.control_mode = null;
      this.reserved = null;
    }
    else {
      if (initObj.hasOwnProperty('time_stamp')) {
        this.time_stamp = initObj.time_stamp
      }
      else {
        this.time_stamp = 0.0;
      }
      if (initObj.hasOwnProperty('actual_joint_position')) {
        this.actual_joint_position = initObj.actual_joint_position
      }
      else {
        this.actual_joint_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_joint_position_abs')) {
        this.actual_joint_position_abs = initObj.actual_joint_position_abs
      }
      else {
        this.actual_joint_position_abs = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_joint_velocity')) {
        this.actual_joint_velocity = initObj.actual_joint_velocity
      }
      else {
        this.actual_joint_velocity = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_joint_velocity_abs')) {
        this.actual_joint_velocity_abs = initObj.actual_joint_velocity_abs
      }
      else {
        this.actual_joint_velocity_abs = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_tcp_position')) {
        this.actual_tcp_position = initObj.actual_tcp_position
      }
      else {
        this.actual_tcp_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_tcp_velocity')) {
        this.actual_tcp_velocity = initObj.actual_tcp_velocity
      }
      else {
        this.actual_tcp_velocity = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_flange_position')) {
        this.actual_flange_position = initObj.actual_flange_position
      }
      else {
        this.actual_flange_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_flange_velocity')) {
        this.actual_flange_velocity = initObj.actual_flange_velocity
      }
      else {
        this.actual_flange_velocity = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_motor_torque')) {
        this.actual_motor_torque = initObj.actual_motor_torque
      }
      else {
        this.actual_motor_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_joint_torque')) {
        this.actual_joint_torque = initObj.actual_joint_torque
      }
      else {
        this.actual_joint_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('raw_joint_torque')) {
        this.raw_joint_torque = initObj.raw_joint_torque
      }
      else {
        this.raw_joint_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('raw_force_torque')) {
        this.raw_force_torque = initObj.raw_force_torque
      }
      else {
        this.raw_force_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('external_joint_torque')) {
        this.external_joint_torque = initObj.external_joint_torque
      }
      else {
        this.external_joint_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('external_tcp_force')) {
        this.external_tcp_force = initObj.external_tcp_force
      }
      else {
        this.external_tcp_force = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_joint_position')) {
        this.target_joint_position = initObj.target_joint_position
      }
      else {
        this.target_joint_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_joint_velocity')) {
        this.target_joint_velocity = initObj.target_joint_velocity
      }
      else {
        this.target_joint_velocity = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_joint_acceleration')) {
        this.target_joint_acceleration = initObj.target_joint_acceleration
      }
      else {
        this.target_joint_acceleration = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_motor_torque')) {
        this.target_motor_torque = initObj.target_motor_torque
      }
      else {
        this.target_motor_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_tcp_position')) {
        this.target_tcp_position = initObj.target_tcp_position
      }
      else {
        this.target_tcp_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_tcp_velocity')) {
        this.target_tcp_velocity = initObj.target_tcp_velocity
      }
      else {
        this.target_tcp_velocity = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('jacobian_matrix')) {
        this.jacobian_matrix = initObj.jacobian_matrix
      }
      else {
        this.jacobian_matrix = [];
      }
      if (initObj.hasOwnProperty('gravity_torque')) {
        this.gravity_torque = initObj.gravity_torque
      }
      else {
        this.gravity_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('coriolis_matrix')) {
        this.coriolis_matrix = initObj.coriolis_matrix
      }
      else {
        this.coriolis_matrix = [];
      }
      if (initObj.hasOwnProperty('mass_matrix')) {
        this.mass_matrix = initObj.mass_matrix
      }
      else {
        this.mass_matrix = [];
      }
      if (initObj.hasOwnProperty('solution_space')) {
        this.solution_space = initObj.solution_space
      }
      else {
        this.solution_space = 0;
      }
      if (initObj.hasOwnProperty('singularity')) {
        this.singularity = initObj.singularity
      }
      else {
        this.singularity = 0.0;
      }
      if (initObj.hasOwnProperty('operation_speed_rate')) {
        this.operation_speed_rate = initObj.operation_speed_rate
      }
      else {
        this.operation_speed_rate = 0.0;
      }
      if (initObj.hasOwnProperty('joint_temperature')) {
        this.joint_temperature = initObj.joint_temperature
      }
      else {
        this.joint_temperature = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('controller_digital_input')) {
        this.controller_digital_input = initObj.controller_digital_input
      }
      else {
        this.controller_digital_input = 0;
      }
      if (initObj.hasOwnProperty('controller_digital_output')) {
        this.controller_digital_output = initObj.controller_digital_output
      }
      else {
        this.controller_digital_output = 0;
      }
      if (initObj.hasOwnProperty('controller_analog_input_type')) {
        this.controller_analog_input_type = initObj.controller_analog_input_type
      }
      else {
        this.controller_analog_input_type = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('controller_analog_input')) {
        this.controller_analog_input = initObj.controller_analog_input
      }
      else {
        this.controller_analog_input = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('controller_analog_output_type')) {
        this.controller_analog_output_type = initObj.controller_analog_output_type
      }
      else {
        this.controller_analog_output_type = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('controller_analog_output')) {
        this.controller_analog_output = initObj.controller_analog_output
      }
      else {
        this.controller_analog_output = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('flange_digital_input')) {
        this.flange_digital_input = initObj.flange_digital_input
      }
      else {
        this.flange_digital_input = 0;
      }
      if (initObj.hasOwnProperty('flange_digital_output')) {
        this.flange_digital_output = initObj.flange_digital_output
      }
      else {
        this.flange_digital_output = 0;
      }
      if (initObj.hasOwnProperty('flange_analog_input')) {
        this.flange_analog_input = initObj.flange_analog_input
      }
      else {
        this.flange_analog_input = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('external_encoder_strobe_count')) {
        this.external_encoder_strobe_count = initObj.external_encoder_strobe_count
      }
      else {
        this.external_encoder_strobe_count = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('external_encoder_count')) {
        this.external_encoder_count = initObj.external_encoder_count
      }
      else {
        this.external_encoder_count = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('goal_joint_position')) {
        this.goal_joint_position = initObj.goal_joint_position
      }
      else {
        this.goal_joint_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('goal_tcp_position')) {
        this.goal_tcp_position = initObj.goal_tcp_position
      }
      else {
        this.goal_tcp_position = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('robot_mode')) {
        this.robot_mode = initObj.robot_mode
      }
      else {
        this.robot_mode = 0;
      }
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = 0;
      }
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('reserved')) {
        this.reserved = initObj.reserved
      }
      else {
        this.reserved = new Array(256).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStateRT
    // Serialize message field [time_stamp]
    bufferOffset = _serializer.float64(obj.time_stamp, buffer, bufferOffset);
    // Check that the constant length array field [actual_joint_position] has the right length
    if (obj.actual_joint_position.length !== 6) {
      throw new Error('Unable to serialize array field actual_joint_position - length must be 6')
    }
    // Serialize message field [actual_joint_position]
    bufferOffset = _arraySerializer.float64(obj.actual_joint_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_joint_position_abs] has the right length
    if (obj.actual_joint_position_abs.length !== 6) {
      throw new Error('Unable to serialize array field actual_joint_position_abs - length must be 6')
    }
    // Serialize message field [actual_joint_position_abs]
    bufferOffset = _arraySerializer.float64(obj.actual_joint_position_abs, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_joint_velocity] has the right length
    if (obj.actual_joint_velocity.length !== 6) {
      throw new Error('Unable to serialize array field actual_joint_velocity - length must be 6')
    }
    // Serialize message field [actual_joint_velocity]
    bufferOffset = _arraySerializer.float64(obj.actual_joint_velocity, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_joint_velocity_abs] has the right length
    if (obj.actual_joint_velocity_abs.length !== 6) {
      throw new Error('Unable to serialize array field actual_joint_velocity_abs - length must be 6')
    }
    // Serialize message field [actual_joint_velocity_abs]
    bufferOffset = _arraySerializer.float64(obj.actual_joint_velocity_abs, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_tcp_position] has the right length
    if (obj.actual_tcp_position.length !== 6) {
      throw new Error('Unable to serialize array field actual_tcp_position - length must be 6')
    }
    // Serialize message field [actual_tcp_position]
    bufferOffset = _arraySerializer.float64(obj.actual_tcp_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_tcp_velocity] has the right length
    if (obj.actual_tcp_velocity.length !== 6) {
      throw new Error('Unable to serialize array field actual_tcp_velocity - length must be 6')
    }
    // Serialize message field [actual_tcp_velocity]
    bufferOffset = _arraySerializer.float64(obj.actual_tcp_velocity, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_flange_position] has the right length
    if (obj.actual_flange_position.length !== 6) {
      throw new Error('Unable to serialize array field actual_flange_position - length must be 6')
    }
    // Serialize message field [actual_flange_position]
    bufferOffset = _arraySerializer.float64(obj.actual_flange_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_flange_velocity] has the right length
    if (obj.actual_flange_velocity.length !== 6) {
      throw new Error('Unable to serialize array field actual_flange_velocity - length must be 6')
    }
    // Serialize message field [actual_flange_velocity]
    bufferOffset = _arraySerializer.float64(obj.actual_flange_velocity, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_motor_torque] has the right length
    if (obj.actual_motor_torque.length !== 6) {
      throw new Error('Unable to serialize array field actual_motor_torque - length must be 6')
    }
    // Serialize message field [actual_motor_torque]
    bufferOffset = _arraySerializer.float64(obj.actual_motor_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_joint_torque] has the right length
    if (obj.actual_joint_torque.length !== 6) {
      throw new Error('Unable to serialize array field actual_joint_torque - length must be 6')
    }
    // Serialize message field [actual_joint_torque]
    bufferOffset = _arraySerializer.float64(obj.actual_joint_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [raw_joint_torque] has the right length
    if (obj.raw_joint_torque.length !== 6) {
      throw new Error('Unable to serialize array field raw_joint_torque - length must be 6')
    }
    // Serialize message field [raw_joint_torque]
    bufferOffset = _arraySerializer.float64(obj.raw_joint_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [raw_force_torque] has the right length
    if (obj.raw_force_torque.length !== 6) {
      throw new Error('Unable to serialize array field raw_force_torque - length must be 6')
    }
    // Serialize message field [raw_force_torque]
    bufferOffset = _arraySerializer.float64(obj.raw_force_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [external_joint_torque] has the right length
    if (obj.external_joint_torque.length !== 6) {
      throw new Error('Unable to serialize array field external_joint_torque - length must be 6')
    }
    // Serialize message field [external_joint_torque]
    bufferOffset = _arraySerializer.float64(obj.external_joint_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [external_tcp_force] has the right length
    if (obj.external_tcp_force.length !== 6) {
      throw new Error('Unable to serialize array field external_tcp_force - length must be 6')
    }
    // Serialize message field [external_tcp_force]
    bufferOffset = _arraySerializer.float64(obj.external_tcp_force, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_joint_position] has the right length
    if (obj.target_joint_position.length !== 6) {
      throw new Error('Unable to serialize array field target_joint_position - length must be 6')
    }
    // Serialize message field [target_joint_position]
    bufferOffset = _arraySerializer.float64(obj.target_joint_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_joint_velocity] has the right length
    if (obj.target_joint_velocity.length !== 6) {
      throw new Error('Unable to serialize array field target_joint_velocity - length must be 6')
    }
    // Serialize message field [target_joint_velocity]
    bufferOffset = _arraySerializer.float64(obj.target_joint_velocity, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_joint_acceleration] has the right length
    if (obj.target_joint_acceleration.length !== 6) {
      throw new Error('Unable to serialize array field target_joint_acceleration - length must be 6')
    }
    // Serialize message field [target_joint_acceleration]
    bufferOffset = _arraySerializer.float64(obj.target_joint_acceleration, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_motor_torque] has the right length
    if (obj.target_motor_torque.length !== 6) {
      throw new Error('Unable to serialize array field target_motor_torque - length must be 6')
    }
    // Serialize message field [target_motor_torque]
    bufferOffset = _arraySerializer.float64(obj.target_motor_torque, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_tcp_position] has the right length
    if (obj.target_tcp_position.length !== 6) {
      throw new Error('Unable to serialize array field target_tcp_position - length must be 6')
    }
    // Serialize message field [target_tcp_position]
    bufferOffset = _arraySerializer.float64(obj.target_tcp_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_tcp_velocity] has the right length
    if (obj.target_tcp_velocity.length !== 6) {
      throw new Error('Unable to serialize array field target_tcp_velocity - length must be 6')
    }
    // Serialize message field [target_tcp_velocity]
    bufferOffset = _arraySerializer.float64(obj.target_tcp_velocity, buffer, bufferOffset, 6);
    // Serialize message field [jacobian_matrix]
    // Serialize the length for message field [jacobian_matrix]
    bufferOffset = _serializer.uint32(obj.jacobian_matrix.length, buffer, bufferOffset);
    obj.jacobian_matrix.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [gravity_torque] has the right length
    if (obj.gravity_torque.length !== 6) {
      throw new Error('Unable to serialize array field gravity_torque - length must be 6')
    }
    // Serialize message field [gravity_torque]
    bufferOffset = _arraySerializer.float64(obj.gravity_torque, buffer, bufferOffset, 6);
    // Serialize message field [coriolis_matrix]
    // Serialize the length for message field [coriolis_matrix]
    bufferOffset = _serializer.uint32(obj.coriolis_matrix.length, buffer, bufferOffset);
    obj.coriolis_matrix.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [mass_matrix]
    // Serialize the length for message field [mass_matrix]
    bufferOffset = _serializer.uint32(obj.mass_matrix.length, buffer, bufferOffset);
    obj.mass_matrix.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [solution_space]
    bufferOffset = _serializer.uint16(obj.solution_space, buffer, bufferOffset);
    // Serialize message field [singularity]
    bufferOffset = _serializer.float64(obj.singularity, buffer, bufferOffset);
    // Serialize message field [operation_speed_rate]
    bufferOffset = _serializer.float64(obj.operation_speed_rate, buffer, bufferOffset);
    // Check that the constant length array field [joint_temperature] has the right length
    if (obj.joint_temperature.length !== 6) {
      throw new Error('Unable to serialize array field joint_temperature - length must be 6')
    }
    // Serialize message field [joint_temperature]
    bufferOffset = _arraySerializer.float64(obj.joint_temperature, buffer, bufferOffset, 6);
    // Serialize message field [controller_digital_input]
    bufferOffset = _serializer.uint16(obj.controller_digital_input, buffer, bufferOffset);
    // Serialize message field [controller_digital_output]
    bufferOffset = _serializer.uint16(obj.controller_digital_output, buffer, bufferOffset);
    // Check that the constant length array field [controller_analog_input_type] has the right length
    if (obj.controller_analog_input_type.length !== 2) {
      throw new Error('Unable to serialize array field controller_analog_input_type - length must be 2')
    }
    // Serialize message field [controller_analog_input_type]
    bufferOffset = _arraySerializer.uint8(obj.controller_analog_input_type, buffer, bufferOffset, 2);
    // Check that the constant length array field [controller_analog_input] has the right length
    if (obj.controller_analog_input.length !== 2) {
      throw new Error('Unable to serialize array field controller_analog_input - length must be 2')
    }
    // Serialize message field [controller_analog_input]
    bufferOffset = _arraySerializer.float64(obj.controller_analog_input, buffer, bufferOffset, 2);
    // Check that the constant length array field [controller_analog_output_type] has the right length
    if (obj.controller_analog_output_type.length !== 2) {
      throw new Error('Unable to serialize array field controller_analog_output_type - length must be 2')
    }
    // Serialize message field [controller_analog_output_type]
    bufferOffset = _arraySerializer.uint8(obj.controller_analog_output_type, buffer, bufferOffset, 2);
    // Check that the constant length array field [controller_analog_output] has the right length
    if (obj.controller_analog_output.length !== 2) {
      throw new Error('Unable to serialize array field controller_analog_output - length must be 2')
    }
    // Serialize message field [controller_analog_output]
    bufferOffset = _arraySerializer.float64(obj.controller_analog_output, buffer, bufferOffset, 2);
    // Serialize message field [flange_digital_input]
    bufferOffset = _serializer.uint8(obj.flange_digital_input, buffer, bufferOffset);
    // Serialize message field [flange_digital_output]
    bufferOffset = _serializer.uint8(obj.flange_digital_output, buffer, bufferOffset);
    // Check that the constant length array field [flange_analog_input] has the right length
    if (obj.flange_analog_input.length !== 4) {
      throw new Error('Unable to serialize array field flange_analog_input - length must be 4')
    }
    // Serialize message field [flange_analog_input]
    bufferOffset = _arraySerializer.float64(obj.flange_analog_input, buffer, bufferOffset, 4);
    // Check that the constant length array field [external_encoder_strobe_count] has the right length
    if (obj.external_encoder_strobe_count.length !== 2) {
      throw new Error('Unable to serialize array field external_encoder_strobe_count - length must be 2')
    }
    // Serialize message field [external_encoder_strobe_count]
    bufferOffset = _arraySerializer.uint8(obj.external_encoder_strobe_count, buffer, bufferOffset, 2);
    // Check that the constant length array field [external_encoder_count] has the right length
    if (obj.external_encoder_count.length !== 2) {
      throw new Error('Unable to serialize array field external_encoder_count - length must be 2')
    }
    // Serialize message field [external_encoder_count]
    bufferOffset = _arraySerializer.uint16(obj.external_encoder_count, buffer, bufferOffset, 2);
    // Check that the constant length array field [goal_joint_position] has the right length
    if (obj.goal_joint_position.length !== 6) {
      throw new Error('Unable to serialize array field goal_joint_position - length must be 6')
    }
    // Serialize message field [goal_joint_position]
    bufferOffset = _arraySerializer.float64(obj.goal_joint_position, buffer, bufferOffset, 6);
    // Check that the constant length array field [goal_tcp_position] has the right length
    if (obj.goal_tcp_position.length !== 6) {
      throw new Error('Unable to serialize array field goal_tcp_position - length must be 6')
    }
    // Serialize message field [goal_tcp_position]
    bufferOffset = _arraySerializer.float64(obj.goal_tcp_position, buffer, bufferOffset, 6);
    // Serialize message field [robot_mode]
    bufferOffset = _serializer.uint8(obj.robot_mode, buffer, bufferOffset);
    // Serialize message field [robot_state]
    bufferOffset = _serializer.uint8(obj.robot_state, buffer, bufferOffset);
    // Serialize message field [control_mode]
    bufferOffset = _serializer.uint16(obj.control_mode, buffer, bufferOffset);
    // Check that the constant length array field [reserved] has the right length
    if (obj.reserved.length !== 256) {
      throw new Error('Unable to serialize array field reserved - length must be 256')
    }
    // Serialize message field [reserved]
    bufferOffset = _arraySerializer.uint8(obj.reserved, buffer, bufferOffset, 256);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStateRT
    let len;
    let data = new RobotStateRT(null);
    // Deserialize message field [time_stamp]
    data.time_stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [actual_joint_position]
    data.actual_joint_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_joint_position_abs]
    data.actual_joint_position_abs = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_joint_velocity]
    data.actual_joint_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_joint_velocity_abs]
    data.actual_joint_velocity_abs = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_tcp_position]
    data.actual_tcp_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_tcp_velocity]
    data.actual_tcp_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_flange_position]
    data.actual_flange_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_flange_velocity]
    data.actual_flange_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_motor_torque]
    data.actual_motor_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_joint_torque]
    data.actual_joint_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [raw_joint_torque]
    data.raw_joint_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [raw_force_torque]
    data.raw_force_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [external_joint_torque]
    data.external_joint_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [external_tcp_force]
    data.external_tcp_force = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_joint_position]
    data.target_joint_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_joint_velocity]
    data.target_joint_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_joint_acceleration]
    data.target_joint_acceleration = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_motor_torque]
    data.target_motor_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_tcp_position]
    data.target_tcp_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_tcp_velocity]
    data.target_tcp_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [jacobian_matrix]
    // Deserialize array length for message field [jacobian_matrix]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.jacobian_matrix = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.jacobian_matrix[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [gravity_torque]
    data.gravity_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [coriolis_matrix]
    // Deserialize array length for message field [coriolis_matrix]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.coriolis_matrix = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.coriolis_matrix[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [mass_matrix]
    // Deserialize array length for message field [mass_matrix]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mass_matrix = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mass_matrix[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [solution_space]
    data.solution_space = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [singularity]
    data.singularity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [operation_speed_rate]
    data.operation_speed_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_temperature]
    data.joint_temperature = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [controller_digital_input]
    data.controller_digital_input = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [controller_digital_output]
    data.controller_digital_output = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [controller_analog_input_type]
    data.controller_analog_input_type = _arrayDeserializer.uint8(buffer, bufferOffset, 2)
    // Deserialize message field [controller_analog_input]
    data.controller_analog_input = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [controller_analog_output_type]
    data.controller_analog_output_type = _arrayDeserializer.uint8(buffer, bufferOffset, 2)
    // Deserialize message field [controller_analog_output]
    data.controller_analog_output = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [flange_digital_input]
    data.flange_digital_input = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flange_digital_output]
    data.flange_digital_output = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flange_analog_input]
    data.flange_analog_input = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [external_encoder_strobe_count]
    data.external_encoder_strobe_count = _arrayDeserializer.uint8(buffer, bufferOffset, 2)
    // Deserialize message field [external_encoder_count]
    data.external_encoder_count = _arrayDeserializer.uint16(buffer, bufferOffset, 2)
    // Deserialize message field [goal_joint_position]
    data.goal_joint_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [goal_tcp_position]
    data.goal_tcp_position = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [robot_mode]
    data.robot_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [robot_state]
    data.robot_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved]
    data.reserved = _arrayDeserializer.uint8(buffer, bufferOffset, 256)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.jacobian_matrix.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.coriolis_matrix.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.mass_matrix.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 1530;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/RobotStateRT';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fc4bdbba50c12ddf9126a58ad825fc4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # timestamp at the data of data acquisition
    float64                      time_stamp
    # actual joint position from incremental encoder at motor side(used for control) [deg]
    float64[6]                   actual_joint_position
    # actual joint position from absolute encoder at link side (used for exact link position) [deg]
    float64[6]                   actual_joint_position_abs
    # actual joint velocity from incremental encoder at motor side [deg/s]
    float64[6]                   actual_joint_velocity
    # actual joint velocity from absolute encoder at link side [deg/s]
    float64[6]                   actual_joint_velocity_abs
    # actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
    float64[6]                   actual_tcp_position
    # actual robot tcp velocity w.r.t. base coordinates [mm, deg/s]
    float64[6]                   actual_tcp_velocity
    # actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
    float64[6]                   actual_flange_position
    # robot flange velocity w.r.t. base coordinates [mm, deg/s]
    float64[6]                   actual_flange_velocity
    # actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm]
    float64[6]                   actual_motor_torque
    # estimated joint torque by robot controller [Nm]
    float64[6]                   actual_joint_torque
    # calibrated joint torque sensor data [Nm]
    float64[6]                   raw_joint_torque
    # calibrated force torque sensor data w.r.t. flange coordinates [N, Nm]
    float64[6]                   raw_force_torque
    # estimated external joint torque [Nm]
    float64[6]                   external_joint_torque
    # estimated tcp force w.r.t. base coordinates [N, Nm] 
    float64[6]                   external_tcp_force
    # target joint position [deg]
    float64[6]                   target_joint_position
    # target joint velocity [deg/s]
    float64[6]                   target_joint_velocity
    # target joint acceleration [deg/s^2] 
    float64[6]                   target_joint_acceleration
    # target motor torque [Nm] 
    float64[6]                   target_motor_torque
    # target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] 
    float64[6]                   target_tcp_position
    # target tcp velocity w.r.t. base coordinates [mm, deg/s]
    float64[6]                   target_tcp_velocity
    # jacobian matrix=J(q) w.r.t. base coordinates
    std_msgs/Float64MultiArray[] jacobian_matrix
    # gravity torque=g(q) [Nm]
    float64[6]                   gravity_torque
    # coriolis matrix=C(q,q_dot)  [6][6]
    std_msgs/Float64MultiArray[] coriolis_matrix
    # mass matrix=M(q) [6][6]
    std_msgs/Float64MultiArray[] mass_matrix
    # robot configuration 
    uint16                       solution_space
    # minimum singular value 
    float64                      singularity
    # current operation speed rate(1~100 %) 
    float64                      operation_speed_rate
    # joint temperature(celsius) 
    float64[6]                   joint_temperature
    # controller digital input(16 channel) 
    uint16                       controller_digital_input
    # controller digital output(16 channel) 
    uint16                       controller_digital_output
    # controller analog input type(2 channel) 
    uint8[2]                      controller_analog_input_type
    # controller analog input(2 channel) 
    float64[2]                   controller_analog_input
    # controller analog output type(2 channel) 
    uint8[2]                     controller_analog_output_type
    # controller analog output(2 channel) 
    float64[2]                   controller_analog_output
    # flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) 
    uint8                        flange_digital_input
    # flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) 
    uint8                        flange_digital_output
    # flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) 
    float64[4]                   flange_analog_input
    # strobe count(increased by 1 when detecting setting edge) 
    uint8[2]                     external_encoder_strobe_count
    # external encoder count 
    uint16[2]                    external_encoder_count
    # final goal joint position (reserved) 
    float64[6]                   goal_joint_position
    # final goal tcp position (reserved) 
    float64[6]                   goal_tcp_position
    # ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) 
    uint8                        robot_mode
    # STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, 
    uint8                        robot_state
    # position control mode, torque mode 
    uint16                       control_mode
    # Reserved 
    uint8[256]                   reserved
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotStateRT(null);
    if (msg.time_stamp !== undefined) {
      resolved.time_stamp = msg.time_stamp;
    }
    else {
      resolved.time_stamp = 0.0
    }

    if (msg.actual_joint_position !== undefined) {
      resolved.actual_joint_position = msg.actual_joint_position;
    }
    else {
      resolved.actual_joint_position = new Array(6).fill(0)
    }

    if (msg.actual_joint_position_abs !== undefined) {
      resolved.actual_joint_position_abs = msg.actual_joint_position_abs;
    }
    else {
      resolved.actual_joint_position_abs = new Array(6).fill(0)
    }

    if (msg.actual_joint_velocity !== undefined) {
      resolved.actual_joint_velocity = msg.actual_joint_velocity;
    }
    else {
      resolved.actual_joint_velocity = new Array(6).fill(0)
    }

    if (msg.actual_joint_velocity_abs !== undefined) {
      resolved.actual_joint_velocity_abs = msg.actual_joint_velocity_abs;
    }
    else {
      resolved.actual_joint_velocity_abs = new Array(6).fill(0)
    }

    if (msg.actual_tcp_position !== undefined) {
      resolved.actual_tcp_position = msg.actual_tcp_position;
    }
    else {
      resolved.actual_tcp_position = new Array(6).fill(0)
    }

    if (msg.actual_tcp_velocity !== undefined) {
      resolved.actual_tcp_velocity = msg.actual_tcp_velocity;
    }
    else {
      resolved.actual_tcp_velocity = new Array(6).fill(0)
    }

    if (msg.actual_flange_position !== undefined) {
      resolved.actual_flange_position = msg.actual_flange_position;
    }
    else {
      resolved.actual_flange_position = new Array(6).fill(0)
    }

    if (msg.actual_flange_velocity !== undefined) {
      resolved.actual_flange_velocity = msg.actual_flange_velocity;
    }
    else {
      resolved.actual_flange_velocity = new Array(6).fill(0)
    }

    if (msg.actual_motor_torque !== undefined) {
      resolved.actual_motor_torque = msg.actual_motor_torque;
    }
    else {
      resolved.actual_motor_torque = new Array(6).fill(0)
    }

    if (msg.actual_joint_torque !== undefined) {
      resolved.actual_joint_torque = msg.actual_joint_torque;
    }
    else {
      resolved.actual_joint_torque = new Array(6).fill(0)
    }

    if (msg.raw_joint_torque !== undefined) {
      resolved.raw_joint_torque = msg.raw_joint_torque;
    }
    else {
      resolved.raw_joint_torque = new Array(6).fill(0)
    }

    if (msg.raw_force_torque !== undefined) {
      resolved.raw_force_torque = msg.raw_force_torque;
    }
    else {
      resolved.raw_force_torque = new Array(6).fill(0)
    }

    if (msg.external_joint_torque !== undefined) {
      resolved.external_joint_torque = msg.external_joint_torque;
    }
    else {
      resolved.external_joint_torque = new Array(6).fill(0)
    }

    if (msg.external_tcp_force !== undefined) {
      resolved.external_tcp_force = msg.external_tcp_force;
    }
    else {
      resolved.external_tcp_force = new Array(6).fill(0)
    }

    if (msg.target_joint_position !== undefined) {
      resolved.target_joint_position = msg.target_joint_position;
    }
    else {
      resolved.target_joint_position = new Array(6).fill(0)
    }

    if (msg.target_joint_velocity !== undefined) {
      resolved.target_joint_velocity = msg.target_joint_velocity;
    }
    else {
      resolved.target_joint_velocity = new Array(6).fill(0)
    }

    if (msg.target_joint_acceleration !== undefined) {
      resolved.target_joint_acceleration = msg.target_joint_acceleration;
    }
    else {
      resolved.target_joint_acceleration = new Array(6).fill(0)
    }

    if (msg.target_motor_torque !== undefined) {
      resolved.target_motor_torque = msg.target_motor_torque;
    }
    else {
      resolved.target_motor_torque = new Array(6).fill(0)
    }

    if (msg.target_tcp_position !== undefined) {
      resolved.target_tcp_position = msg.target_tcp_position;
    }
    else {
      resolved.target_tcp_position = new Array(6).fill(0)
    }

    if (msg.target_tcp_velocity !== undefined) {
      resolved.target_tcp_velocity = msg.target_tcp_velocity;
    }
    else {
      resolved.target_tcp_velocity = new Array(6).fill(0)
    }

    if (msg.jacobian_matrix !== undefined) {
      resolved.jacobian_matrix = new Array(msg.jacobian_matrix.length);
      for (let i = 0; i < resolved.jacobian_matrix.length; ++i) {
        resolved.jacobian_matrix[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.jacobian_matrix[i]);
      }
    }
    else {
      resolved.jacobian_matrix = []
    }

    if (msg.gravity_torque !== undefined) {
      resolved.gravity_torque = msg.gravity_torque;
    }
    else {
      resolved.gravity_torque = new Array(6).fill(0)
    }

    if (msg.coriolis_matrix !== undefined) {
      resolved.coriolis_matrix = new Array(msg.coriolis_matrix.length);
      for (let i = 0; i < resolved.coriolis_matrix.length; ++i) {
        resolved.coriolis_matrix[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.coriolis_matrix[i]);
      }
    }
    else {
      resolved.coriolis_matrix = []
    }

    if (msg.mass_matrix !== undefined) {
      resolved.mass_matrix = new Array(msg.mass_matrix.length);
      for (let i = 0; i < resolved.mass_matrix.length; ++i) {
        resolved.mass_matrix[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.mass_matrix[i]);
      }
    }
    else {
      resolved.mass_matrix = []
    }

    if (msg.solution_space !== undefined) {
      resolved.solution_space = msg.solution_space;
    }
    else {
      resolved.solution_space = 0
    }

    if (msg.singularity !== undefined) {
      resolved.singularity = msg.singularity;
    }
    else {
      resolved.singularity = 0.0
    }

    if (msg.operation_speed_rate !== undefined) {
      resolved.operation_speed_rate = msg.operation_speed_rate;
    }
    else {
      resolved.operation_speed_rate = 0.0
    }

    if (msg.joint_temperature !== undefined) {
      resolved.joint_temperature = msg.joint_temperature;
    }
    else {
      resolved.joint_temperature = new Array(6).fill(0)
    }

    if (msg.controller_digital_input !== undefined) {
      resolved.controller_digital_input = msg.controller_digital_input;
    }
    else {
      resolved.controller_digital_input = 0
    }

    if (msg.controller_digital_output !== undefined) {
      resolved.controller_digital_output = msg.controller_digital_output;
    }
    else {
      resolved.controller_digital_output = 0
    }

    if (msg.controller_analog_input_type !== undefined) {
      resolved.controller_analog_input_type = msg.controller_analog_input_type;
    }
    else {
      resolved.controller_analog_input_type = new Array(2).fill(0)
    }

    if (msg.controller_analog_input !== undefined) {
      resolved.controller_analog_input = msg.controller_analog_input;
    }
    else {
      resolved.controller_analog_input = new Array(2).fill(0)
    }

    if (msg.controller_analog_output_type !== undefined) {
      resolved.controller_analog_output_type = msg.controller_analog_output_type;
    }
    else {
      resolved.controller_analog_output_type = new Array(2).fill(0)
    }

    if (msg.controller_analog_output !== undefined) {
      resolved.controller_analog_output = msg.controller_analog_output;
    }
    else {
      resolved.controller_analog_output = new Array(2).fill(0)
    }

    if (msg.flange_digital_input !== undefined) {
      resolved.flange_digital_input = msg.flange_digital_input;
    }
    else {
      resolved.flange_digital_input = 0
    }

    if (msg.flange_digital_output !== undefined) {
      resolved.flange_digital_output = msg.flange_digital_output;
    }
    else {
      resolved.flange_digital_output = 0
    }

    if (msg.flange_analog_input !== undefined) {
      resolved.flange_analog_input = msg.flange_analog_input;
    }
    else {
      resolved.flange_analog_input = new Array(4).fill(0)
    }

    if (msg.external_encoder_strobe_count !== undefined) {
      resolved.external_encoder_strobe_count = msg.external_encoder_strobe_count;
    }
    else {
      resolved.external_encoder_strobe_count = new Array(2).fill(0)
    }

    if (msg.external_encoder_count !== undefined) {
      resolved.external_encoder_count = msg.external_encoder_count;
    }
    else {
      resolved.external_encoder_count = new Array(2).fill(0)
    }

    if (msg.goal_joint_position !== undefined) {
      resolved.goal_joint_position = msg.goal_joint_position;
    }
    else {
      resolved.goal_joint_position = new Array(6).fill(0)
    }

    if (msg.goal_tcp_position !== undefined) {
      resolved.goal_tcp_position = msg.goal_tcp_position;
    }
    else {
      resolved.goal_tcp_position = new Array(6).fill(0)
    }

    if (msg.robot_mode !== undefined) {
      resolved.robot_mode = msg.robot_mode;
    }
    else {
      resolved.robot_mode = 0
    }

    if (msg.robot_state !== undefined) {
      resolved.robot_state = msg.robot_state;
    }
    else {
      resolved.robot_state = 0
    }

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.reserved !== undefined) {
      resolved.reserved = msg.reserved;
    }
    else {
      resolved.reserved = new Array(256).fill(0)
    }

    return resolved;
    }
};

module.exports = RobotStateRT;
