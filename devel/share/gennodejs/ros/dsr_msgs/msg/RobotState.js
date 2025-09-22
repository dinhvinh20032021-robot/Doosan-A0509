// Auto-generated. Do not edit!

// (in-package dsr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ModbusState = require('./ModbusState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_state = null;
      this.robot_state_str = null;
      this.actual_mode = null;
      this.actual_space = null;
      this.current_posj = null;
      this.current_velj = null;
      this.joint_abs = null;
      this.joint_err = null;
      this.target_posj = null;
      this.target_velj = null;
      this.current_posx = null;
      this.current_tool_posx = null;
      this.current_velx = null;
      this.task_err = null;
      this.target_velx = null;
      this.target_posx = null;
      this.dynamic_tor = null;
      this.actual_jts = null;
      this.actual_ejt = null;
      this.actual_ett = null;
      this.actual_bk = null;
      this.actual_mc = null;
      this.actual_mt = null;
      this.solution_space = null;
      this.sync_time = null;
      this.actual_bt = null;
      this.rotation_matrix = null;
      this.ctrlbox_digital_input = null;
      this.ctrlbox_digital_output = null;
      this.flange_digital_input = null;
      this.flange_digital_output = null;
      this.modbus_state = null;
      this.access_control = null;
      this.homming_completed = null;
      this.tp_initialized = null;
      this.mastering_need = null;
      this.drl_stopped = null;
      this.disconnected = null;
      this.fActualW2B = null;
      this.fCurrentPosW = null;
      this.fCurrentVelW = null;
      this.fWorldETT = null;
      this.fTargetPosW = null;
      this.fTargetVelW = null;
      this.fRotationMatrixWorld = null;
      this.iActualUCN = null;
      this.iParent = null;
      this.fCurrentPosU = null;
      this.fCurrentVelU = null;
      this.fUserETT = null;
      this.fTargetPosU = null;
      this.fTargetVelU = null;
      this.fRotationMatrixUser = null;
      this.fActualAI = null;
      this.bActualSW = null;
      this.bActualSI = null;
      this.iActualAT = null;
      this.fTargetAO = null;
      this.iTargetAT = null;
      this.bActualES = null;
      this.iActualED = null;
      this.bActualER = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = 0;
      }
      if (initObj.hasOwnProperty('robot_state_str')) {
        this.robot_state_str = initObj.robot_state_str
      }
      else {
        this.robot_state_str = '';
      }
      if (initObj.hasOwnProperty('actual_mode')) {
        this.actual_mode = initObj.actual_mode
      }
      else {
        this.actual_mode = 0;
      }
      if (initObj.hasOwnProperty('actual_space')) {
        this.actual_space = initObj.actual_space
      }
      else {
        this.actual_space = 0;
      }
      if (initObj.hasOwnProperty('current_posj')) {
        this.current_posj = initObj.current_posj
      }
      else {
        this.current_posj = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('current_velj')) {
        this.current_velj = initObj.current_velj
      }
      else {
        this.current_velj = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_abs')) {
        this.joint_abs = initObj.joint_abs
      }
      else {
        this.joint_abs = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_err')) {
        this.joint_err = initObj.joint_err
      }
      else {
        this.joint_err = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_posj')) {
        this.target_posj = initObj.target_posj
      }
      else {
        this.target_posj = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_velj')) {
        this.target_velj = initObj.target_velj
      }
      else {
        this.target_velj = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('current_posx')) {
        this.current_posx = initObj.current_posx
      }
      else {
        this.current_posx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('current_tool_posx')) {
        this.current_tool_posx = initObj.current_tool_posx
      }
      else {
        this.current_tool_posx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('current_velx')) {
        this.current_velx = initObj.current_velx
      }
      else {
        this.current_velx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('task_err')) {
        this.task_err = initObj.task_err
      }
      else {
        this.task_err = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_velx')) {
        this.target_velx = initObj.target_velx
      }
      else {
        this.target_velx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('target_posx')) {
        this.target_posx = initObj.target_posx
      }
      else {
        this.target_posx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('dynamic_tor')) {
        this.dynamic_tor = initObj.dynamic_tor
      }
      else {
        this.dynamic_tor = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_jts')) {
        this.actual_jts = initObj.actual_jts
      }
      else {
        this.actual_jts = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_ejt')) {
        this.actual_ejt = initObj.actual_ejt
      }
      else {
        this.actual_ejt = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_ett')) {
        this.actual_ett = initObj.actual_ett
      }
      else {
        this.actual_ett = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_bk')) {
        this.actual_bk = initObj.actual_bk
      }
      else {
        this.actual_bk = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_mc')) {
        this.actual_mc = initObj.actual_mc
      }
      else {
        this.actual_mc = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('actual_mt')) {
        this.actual_mt = initObj.actual_mt
      }
      else {
        this.actual_mt = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('solution_space')) {
        this.solution_space = initObj.solution_space
      }
      else {
        this.solution_space = 0;
      }
      if (initObj.hasOwnProperty('sync_time')) {
        this.sync_time = initObj.sync_time
      }
      else {
        this.sync_time = 0.0;
      }
      if (initObj.hasOwnProperty('actual_bt')) {
        this.actual_bt = initObj.actual_bt
      }
      else {
        this.actual_bt = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('rotation_matrix')) {
        this.rotation_matrix = initObj.rotation_matrix
      }
      else {
        this.rotation_matrix = [];
      }
      if (initObj.hasOwnProperty('ctrlbox_digital_input')) {
        this.ctrlbox_digital_input = initObj.ctrlbox_digital_input
      }
      else {
        this.ctrlbox_digital_input = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('ctrlbox_digital_output')) {
        this.ctrlbox_digital_output = initObj.ctrlbox_digital_output
      }
      else {
        this.ctrlbox_digital_output = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('flange_digital_input')) {
        this.flange_digital_input = initObj.flange_digital_input
      }
      else {
        this.flange_digital_input = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('flange_digital_output')) {
        this.flange_digital_output = initObj.flange_digital_output
      }
      else {
        this.flange_digital_output = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('modbus_state')) {
        this.modbus_state = initObj.modbus_state
      }
      else {
        this.modbus_state = [];
      }
      if (initObj.hasOwnProperty('access_control')) {
        this.access_control = initObj.access_control
      }
      else {
        this.access_control = 0;
      }
      if (initObj.hasOwnProperty('homming_completed')) {
        this.homming_completed = initObj.homming_completed
      }
      else {
        this.homming_completed = false;
      }
      if (initObj.hasOwnProperty('tp_initialized')) {
        this.tp_initialized = initObj.tp_initialized
      }
      else {
        this.tp_initialized = false;
      }
      if (initObj.hasOwnProperty('mastering_need')) {
        this.mastering_need = initObj.mastering_need
      }
      else {
        this.mastering_need = 0;
      }
      if (initObj.hasOwnProperty('drl_stopped')) {
        this.drl_stopped = initObj.drl_stopped
      }
      else {
        this.drl_stopped = false;
      }
      if (initObj.hasOwnProperty('disconnected')) {
        this.disconnected = initObj.disconnected
      }
      else {
        this.disconnected = false;
      }
      if (initObj.hasOwnProperty('fActualW2B')) {
        this.fActualW2B = initObj.fActualW2B
      }
      else {
        this.fActualW2B = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fCurrentPosW')) {
        this.fCurrentPosW = initObj.fCurrentPosW
      }
      else {
        this.fCurrentPosW = [];
      }
      if (initObj.hasOwnProperty('fCurrentVelW')) {
        this.fCurrentVelW = initObj.fCurrentVelW
      }
      else {
        this.fCurrentVelW = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fWorldETT')) {
        this.fWorldETT = initObj.fWorldETT
      }
      else {
        this.fWorldETT = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fTargetPosW')) {
        this.fTargetPosW = initObj.fTargetPosW
      }
      else {
        this.fTargetPosW = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fTargetVelW')) {
        this.fTargetVelW = initObj.fTargetVelW
      }
      else {
        this.fTargetVelW = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fRotationMatrixWorld')) {
        this.fRotationMatrixWorld = initObj.fRotationMatrixWorld
      }
      else {
        this.fRotationMatrixWorld = [];
      }
      if (initObj.hasOwnProperty('iActualUCN')) {
        this.iActualUCN = initObj.iActualUCN
      }
      else {
        this.iActualUCN = 0;
      }
      if (initObj.hasOwnProperty('iParent')) {
        this.iParent = initObj.iParent
      }
      else {
        this.iParent = 0;
      }
      if (initObj.hasOwnProperty('fCurrentPosU')) {
        this.fCurrentPosU = initObj.fCurrentPosU
      }
      else {
        this.fCurrentPosU = [];
      }
      if (initObj.hasOwnProperty('fCurrentVelU')) {
        this.fCurrentVelU = initObj.fCurrentVelU
      }
      else {
        this.fCurrentVelU = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fUserETT')) {
        this.fUserETT = initObj.fUserETT
      }
      else {
        this.fUserETT = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fTargetPosU')) {
        this.fTargetPosU = initObj.fTargetPosU
      }
      else {
        this.fTargetPosU = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fTargetVelU')) {
        this.fTargetVelU = initObj.fTargetVelU
      }
      else {
        this.fTargetVelU = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('fRotationMatrixUser')) {
        this.fRotationMatrixUser = initObj.fRotationMatrixUser
      }
      else {
        this.fRotationMatrixUser = [];
      }
      if (initObj.hasOwnProperty('fActualAI')) {
        this.fActualAI = initObj.fActualAI
      }
      else {
        this.fActualAI = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('bActualSW')) {
        this.bActualSW = initObj.bActualSW
      }
      else {
        this.bActualSW = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('bActualSI')) {
        this.bActualSI = initObj.bActualSI
      }
      else {
        this.bActualSI = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('iActualAT')) {
        this.iActualAT = initObj.iActualAT
      }
      else {
        this.iActualAT = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('fTargetAO')) {
        this.fTargetAO = initObj.fTargetAO
      }
      else {
        this.fTargetAO = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('iTargetAT')) {
        this.iTargetAT = initObj.iTargetAT
      }
      else {
        this.iTargetAT = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('bActualES')) {
        this.bActualES = initObj.bActualES
      }
      else {
        this.bActualES = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('iActualED')) {
        this.iActualED = initObj.iActualED
      }
      else {
        this.iActualED = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('bActualER')) {
        this.bActualER = initObj.bActualER
      }
      else {
        this.bActualER = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [robot_state]
    bufferOffset = _serializer.int32(obj.robot_state, buffer, bufferOffset);
    // Serialize message field [robot_state_str]
    bufferOffset = _serializer.string(obj.robot_state_str, buffer, bufferOffset);
    // Serialize message field [actual_mode]
    bufferOffset = _serializer.int8(obj.actual_mode, buffer, bufferOffset);
    // Serialize message field [actual_space]
    bufferOffset = _serializer.int8(obj.actual_space, buffer, bufferOffset);
    // Check that the constant length array field [current_posj] has the right length
    if (obj.current_posj.length !== 6) {
      throw new Error('Unable to serialize array field current_posj - length must be 6')
    }
    // Serialize message field [current_posj]
    bufferOffset = _arraySerializer.float64(obj.current_posj, buffer, bufferOffset, 6);
    // Check that the constant length array field [current_velj] has the right length
    if (obj.current_velj.length !== 6) {
      throw new Error('Unable to serialize array field current_velj - length must be 6')
    }
    // Serialize message field [current_velj]
    bufferOffset = _arraySerializer.float64(obj.current_velj, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_abs] has the right length
    if (obj.joint_abs.length !== 6) {
      throw new Error('Unable to serialize array field joint_abs - length must be 6')
    }
    // Serialize message field [joint_abs]
    bufferOffset = _arraySerializer.float64(obj.joint_abs, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_err] has the right length
    if (obj.joint_err.length !== 6) {
      throw new Error('Unable to serialize array field joint_err - length must be 6')
    }
    // Serialize message field [joint_err]
    bufferOffset = _arraySerializer.float64(obj.joint_err, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_posj] has the right length
    if (obj.target_posj.length !== 6) {
      throw new Error('Unable to serialize array field target_posj - length must be 6')
    }
    // Serialize message field [target_posj]
    bufferOffset = _arraySerializer.float64(obj.target_posj, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_velj] has the right length
    if (obj.target_velj.length !== 6) {
      throw new Error('Unable to serialize array field target_velj - length must be 6')
    }
    // Serialize message field [target_velj]
    bufferOffset = _arraySerializer.float64(obj.target_velj, buffer, bufferOffset, 6);
    // Check that the constant length array field [current_posx] has the right length
    if (obj.current_posx.length !== 6) {
      throw new Error('Unable to serialize array field current_posx - length must be 6')
    }
    // Serialize message field [current_posx]
    bufferOffset = _arraySerializer.float64(obj.current_posx, buffer, bufferOffset, 6);
    // Check that the constant length array field [current_tool_posx] has the right length
    if (obj.current_tool_posx.length !== 6) {
      throw new Error('Unable to serialize array field current_tool_posx - length must be 6')
    }
    // Serialize message field [current_tool_posx]
    bufferOffset = _arraySerializer.float64(obj.current_tool_posx, buffer, bufferOffset, 6);
    // Check that the constant length array field [current_velx] has the right length
    if (obj.current_velx.length !== 6) {
      throw new Error('Unable to serialize array field current_velx - length must be 6')
    }
    // Serialize message field [current_velx]
    bufferOffset = _arraySerializer.float64(obj.current_velx, buffer, bufferOffset, 6);
    // Check that the constant length array field [task_err] has the right length
    if (obj.task_err.length !== 6) {
      throw new Error('Unable to serialize array field task_err - length must be 6')
    }
    // Serialize message field [task_err]
    bufferOffset = _arraySerializer.float64(obj.task_err, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_velx] has the right length
    if (obj.target_velx.length !== 6) {
      throw new Error('Unable to serialize array field target_velx - length must be 6')
    }
    // Serialize message field [target_velx]
    bufferOffset = _arraySerializer.float64(obj.target_velx, buffer, bufferOffset, 6);
    // Check that the constant length array field [target_posx] has the right length
    if (obj.target_posx.length !== 6) {
      throw new Error('Unable to serialize array field target_posx - length must be 6')
    }
    // Serialize message field [target_posx]
    bufferOffset = _arraySerializer.float64(obj.target_posx, buffer, bufferOffset, 6);
    // Check that the constant length array field [dynamic_tor] has the right length
    if (obj.dynamic_tor.length !== 6) {
      throw new Error('Unable to serialize array field dynamic_tor - length must be 6')
    }
    // Serialize message field [dynamic_tor]
    bufferOffset = _arraySerializer.float64(obj.dynamic_tor, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_jts] has the right length
    if (obj.actual_jts.length !== 6) {
      throw new Error('Unable to serialize array field actual_jts - length must be 6')
    }
    // Serialize message field [actual_jts]
    bufferOffset = _arraySerializer.float64(obj.actual_jts, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_ejt] has the right length
    if (obj.actual_ejt.length !== 6) {
      throw new Error('Unable to serialize array field actual_ejt - length must be 6')
    }
    // Serialize message field [actual_ejt]
    bufferOffset = _arraySerializer.float64(obj.actual_ejt, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_ett] has the right length
    if (obj.actual_ett.length !== 6) {
      throw new Error('Unable to serialize array field actual_ett - length must be 6')
    }
    // Serialize message field [actual_ett]
    bufferOffset = _arraySerializer.float64(obj.actual_ett, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_bk] has the right length
    if (obj.actual_bk.length !== 6) {
      throw new Error('Unable to serialize array field actual_bk - length must be 6')
    }
    // Serialize message field [actual_bk]
    bufferOffset = _arraySerializer.int8(obj.actual_bk, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_mc] has the right length
    if (obj.actual_mc.length !== 6) {
      throw new Error('Unable to serialize array field actual_mc - length must be 6')
    }
    // Serialize message field [actual_mc]
    bufferOffset = _arraySerializer.float64(obj.actual_mc, buffer, bufferOffset, 6);
    // Check that the constant length array field [actual_mt] has the right length
    if (obj.actual_mt.length !== 6) {
      throw new Error('Unable to serialize array field actual_mt - length must be 6')
    }
    // Serialize message field [actual_mt]
    bufferOffset = _arraySerializer.float64(obj.actual_mt, buffer, bufferOffset, 6);
    // Serialize message field [solution_space]
    bufferOffset = _serializer.int8(obj.solution_space, buffer, bufferOffset);
    // Serialize message field [sync_time]
    bufferOffset = _serializer.float64(obj.sync_time, buffer, bufferOffset);
    // Check that the constant length array field [actual_bt] has the right length
    if (obj.actual_bt.length !== 5) {
      throw new Error('Unable to serialize array field actual_bt - length must be 5')
    }
    // Serialize message field [actual_bt]
    bufferOffset = _arraySerializer.int8(obj.actual_bt, buffer, bufferOffset, 5);
    // Serialize message field [rotation_matrix]
    // Serialize the length for message field [rotation_matrix]
    bufferOffset = _serializer.uint32(obj.rotation_matrix.length, buffer, bufferOffset);
    obj.rotation_matrix.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [ctrlbox_digital_input] has the right length
    if (obj.ctrlbox_digital_input.length !== 16) {
      throw new Error('Unable to serialize array field ctrlbox_digital_input - length must be 16')
    }
    // Serialize message field [ctrlbox_digital_input]
    bufferOffset = _arraySerializer.int8(obj.ctrlbox_digital_input, buffer, bufferOffset, 16);
    // Check that the constant length array field [ctrlbox_digital_output] has the right length
    if (obj.ctrlbox_digital_output.length !== 16) {
      throw new Error('Unable to serialize array field ctrlbox_digital_output - length must be 16')
    }
    // Serialize message field [ctrlbox_digital_output]
    bufferOffset = _arraySerializer.int8(obj.ctrlbox_digital_output, buffer, bufferOffset, 16);
    // Check that the constant length array field [flange_digital_input] has the right length
    if (obj.flange_digital_input.length !== 6) {
      throw new Error('Unable to serialize array field flange_digital_input - length must be 6')
    }
    // Serialize message field [flange_digital_input]
    bufferOffset = _arraySerializer.int8(obj.flange_digital_input, buffer, bufferOffset, 6);
    // Check that the constant length array field [flange_digital_output] has the right length
    if (obj.flange_digital_output.length !== 6) {
      throw new Error('Unable to serialize array field flange_digital_output - length must be 6')
    }
    // Serialize message field [flange_digital_output]
    bufferOffset = _arraySerializer.int8(obj.flange_digital_output, buffer, bufferOffset, 6);
    // Serialize message field [modbus_state]
    // Serialize the length for message field [modbus_state]
    bufferOffset = _serializer.uint32(obj.modbus_state.length, buffer, bufferOffset);
    obj.modbus_state.forEach((val) => {
      bufferOffset = ModbusState.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [access_control]
    bufferOffset = _serializer.int32(obj.access_control, buffer, bufferOffset);
    // Serialize message field [homming_completed]
    bufferOffset = _serializer.bool(obj.homming_completed, buffer, bufferOffset);
    // Serialize message field [tp_initialized]
    bufferOffset = _serializer.bool(obj.tp_initialized, buffer, bufferOffset);
    // Serialize message field [mastering_need]
    bufferOffset = _serializer.int8(obj.mastering_need, buffer, bufferOffset);
    // Serialize message field [drl_stopped]
    bufferOffset = _serializer.bool(obj.drl_stopped, buffer, bufferOffset);
    // Serialize message field [disconnected]
    bufferOffset = _serializer.bool(obj.disconnected, buffer, bufferOffset);
    // Check that the constant length array field [fActualW2B] has the right length
    if (obj.fActualW2B.length !== 6) {
      throw new Error('Unable to serialize array field fActualW2B - length must be 6')
    }
    // Serialize message field [fActualW2B]
    bufferOffset = _arraySerializer.float64(obj.fActualW2B, buffer, bufferOffset, 6);
    // Serialize message field [fCurrentPosW]
    // Serialize the length for message field [fCurrentPosW]
    bufferOffset = _serializer.uint32(obj.fCurrentPosW.length, buffer, bufferOffset);
    obj.fCurrentPosW.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [fCurrentVelW] has the right length
    if (obj.fCurrentVelW.length !== 6) {
      throw new Error('Unable to serialize array field fCurrentVelW - length must be 6')
    }
    // Serialize message field [fCurrentVelW]
    bufferOffset = _arraySerializer.float64(obj.fCurrentVelW, buffer, bufferOffset, 6);
    // Check that the constant length array field [fWorldETT] has the right length
    if (obj.fWorldETT.length !== 6) {
      throw new Error('Unable to serialize array field fWorldETT - length must be 6')
    }
    // Serialize message field [fWorldETT]
    bufferOffset = _arraySerializer.float64(obj.fWorldETT, buffer, bufferOffset, 6);
    // Check that the constant length array field [fTargetPosW] has the right length
    if (obj.fTargetPosW.length !== 6) {
      throw new Error('Unable to serialize array field fTargetPosW - length must be 6')
    }
    // Serialize message field [fTargetPosW]
    bufferOffset = _arraySerializer.float64(obj.fTargetPosW, buffer, bufferOffset, 6);
    // Check that the constant length array field [fTargetVelW] has the right length
    if (obj.fTargetVelW.length !== 6) {
      throw new Error('Unable to serialize array field fTargetVelW - length must be 6')
    }
    // Serialize message field [fTargetVelW]
    bufferOffset = _arraySerializer.float64(obj.fTargetVelW, buffer, bufferOffset, 6);
    // Serialize message field [fRotationMatrixWorld]
    // Serialize the length for message field [fRotationMatrixWorld]
    bufferOffset = _serializer.uint32(obj.fRotationMatrixWorld.length, buffer, bufferOffset);
    obj.fRotationMatrixWorld.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [iActualUCN]
    bufferOffset = _serializer.int8(obj.iActualUCN, buffer, bufferOffset);
    // Serialize message field [iParent]
    bufferOffset = _serializer.int8(obj.iParent, buffer, bufferOffset);
    // Serialize message field [fCurrentPosU]
    // Serialize the length for message field [fCurrentPosU]
    bufferOffset = _serializer.uint32(obj.fCurrentPosU.length, buffer, bufferOffset);
    obj.fCurrentPosU.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [fCurrentVelU] has the right length
    if (obj.fCurrentVelU.length !== 6) {
      throw new Error('Unable to serialize array field fCurrentVelU - length must be 6')
    }
    // Serialize message field [fCurrentVelU]
    bufferOffset = _arraySerializer.float64(obj.fCurrentVelU, buffer, bufferOffset, 6);
    // Check that the constant length array field [fUserETT] has the right length
    if (obj.fUserETT.length !== 6) {
      throw new Error('Unable to serialize array field fUserETT - length must be 6')
    }
    // Serialize message field [fUserETT]
    bufferOffset = _arraySerializer.float64(obj.fUserETT, buffer, bufferOffset, 6);
    // Check that the constant length array field [fTargetPosU] has the right length
    if (obj.fTargetPosU.length !== 6) {
      throw new Error('Unable to serialize array field fTargetPosU - length must be 6')
    }
    // Serialize message field [fTargetPosU]
    bufferOffset = _arraySerializer.float64(obj.fTargetPosU, buffer, bufferOffset, 6);
    // Check that the constant length array field [fTargetVelU] has the right length
    if (obj.fTargetVelU.length !== 6) {
      throw new Error('Unable to serialize array field fTargetVelU - length must be 6')
    }
    // Serialize message field [fTargetVelU]
    bufferOffset = _arraySerializer.float64(obj.fTargetVelU, buffer, bufferOffset, 6);
    // Serialize message field [fRotationMatrixUser]
    // Serialize the length for message field [fRotationMatrixUser]
    bufferOffset = _serializer.uint32(obj.fRotationMatrixUser.length, buffer, bufferOffset);
    obj.fRotationMatrixUser.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [fActualAI] has the right length
    if (obj.fActualAI.length !== 6) {
      throw new Error('Unable to serialize array field fActualAI - length must be 6')
    }
    // Serialize message field [fActualAI]
    bufferOffset = _arraySerializer.float64(obj.fActualAI, buffer, bufferOffset, 6);
    // Check that the constant length array field [bActualSW] has the right length
    if (obj.bActualSW.length !== 3) {
      throw new Error('Unable to serialize array field bActualSW - length must be 3')
    }
    // Serialize message field [bActualSW]
    bufferOffset = _arraySerializer.bool(obj.bActualSW, buffer, bufferOffset, 3);
    // Check that the constant length array field [bActualSI] has the right length
    if (obj.bActualSI.length !== 2) {
      throw new Error('Unable to serialize array field bActualSI - length must be 2')
    }
    // Serialize message field [bActualSI]
    bufferOffset = _arraySerializer.bool(obj.bActualSI, buffer, bufferOffset, 2);
    // Check that the constant length array field [iActualAT] has the right length
    if (obj.iActualAT.length !== 2) {
      throw new Error('Unable to serialize array field iActualAT - length must be 2')
    }
    // Serialize message field [iActualAT]
    bufferOffset = _arraySerializer.int8(obj.iActualAT, buffer, bufferOffset, 2);
    // Check that the constant length array field [fTargetAO] has the right length
    if (obj.fTargetAO.length !== 2) {
      throw new Error('Unable to serialize array field fTargetAO - length must be 2')
    }
    // Serialize message field [fTargetAO]
    bufferOffset = _arraySerializer.float64(obj.fTargetAO, buffer, bufferOffset, 2);
    // Check that the constant length array field [iTargetAT] has the right length
    if (obj.iTargetAT.length !== 2) {
      throw new Error('Unable to serialize array field iTargetAT - length must be 2')
    }
    // Serialize message field [iTargetAT]
    bufferOffset = _arraySerializer.int8(obj.iTargetAT, buffer, bufferOffset, 2);
    // Check that the constant length array field [bActualES] has the right length
    if (obj.bActualES.length !== 2) {
      throw new Error('Unable to serialize array field bActualES - length must be 2')
    }
    // Serialize message field [bActualES]
    bufferOffset = _arraySerializer.bool(obj.bActualES, buffer, bufferOffset, 2);
    // Check that the constant length array field [iActualED] has the right length
    if (obj.iActualED.length !== 2) {
      throw new Error('Unable to serialize array field iActualED - length must be 2')
    }
    // Serialize message field [iActualED]
    bufferOffset = _arraySerializer.int8(obj.iActualED, buffer, bufferOffset, 2);
    // Check that the constant length array field [bActualER] has the right length
    if (obj.bActualER.length !== 2) {
      throw new Error('Unable to serialize array field bActualER - length must be 2')
    }
    // Serialize message field [bActualER]
    bufferOffset = _arraySerializer.bool(obj.bActualER, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [robot_state]
    data.robot_state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [robot_state_str]
    data.robot_state_str = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [actual_mode]
    data.actual_mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [actual_space]
    data.actual_space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [current_posj]
    data.current_posj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [current_velj]
    data.current_velj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_abs]
    data.joint_abs = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_err]
    data.joint_err = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_posj]
    data.target_posj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_velj]
    data.target_velj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [current_posx]
    data.current_posx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [current_tool_posx]
    data.current_tool_posx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [current_velx]
    data.current_velx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [task_err]
    data.task_err = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_velx]
    data.target_velx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [target_posx]
    data.target_posx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [dynamic_tor]
    data.dynamic_tor = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_jts]
    data.actual_jts = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_ejt]
    data.actual_ejt = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_ett]
    data.actual_ett = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_bk]
    data.actual_bk = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [actual_mc]
    data.actual_mc = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [actual_mt]
    data.actual_mt = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [solution_space]
    data.solution_space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [sync_time]
    data.sync_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [actual_bt]
    data.actual_bt = _arrayDeserializer.int8(buffer, bufferOffset, 5)
    // Deserialize message field [rotation_matrix]
    // Deserialize array length for message field [rotation_matrix]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rotation_matrix = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rotation_matrix[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ctrlbox_digital_input]
    data.ctrlbox_digital_input = _arrayDeserializer.int8(buffer, bufferOffset, 16)
    // Deserialize message field [ctrlbox_digital_output]
    data.ctrlbox_digital_output = _arrayDeserializer.int8(buffer, bufferOffset, 16)
    // Deserialize message field [flange_digital_input]
    data.flange_digital_input = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [flange_digital_output]
    data.flange_digital_output = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [modbus_state]
    // Deserialize array length for message field [modbus_state]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.modbus_state = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.modbus_state[i] = ModbusState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [access_control]
    data.access_control = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [homming_completed]
    data.homming_completed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tp_initialized]
    data.tp_initialized = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mastering_need]
    data.mastering_need = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [drl_stopped]
    data.drl_stopped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [disconnected]
    data.disconnected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fActualW2B]
    data.fActualW2B = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fCurrentPosW]
    // Deserialize array length for message field [fCurrentPosW]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.fCurrentPosW = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.fCurrentPosW[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [fCurrentVelW]
    data.fCurrentVelW = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fWorldETT]
    data.fWorldETT = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fTargetPosW]
    data.fTargetPosW = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fTargetVelW]
    data.fTargetVelW = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fRotationMatrixWorld]
    // Deserialize array length for message field [fRotationMatrixWorld]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.fRotationMatrixWorld = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.fRotationMatrixWorld[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [iActualUCN]
    data.iActualUCN = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [iParent]
    data.iParent = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fCurrentPosU]
    // Deserialize array length for message field [fCurrentPosU]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.fCurrentPosU = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.fCurrentPosU[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [fCurrentVelU]
    data.fCurrentVelU = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fUserETT]
    data.fUserETT = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fTargetPosU]
    data.fTargetPosU = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fTargetVelU]
    data.fTargetVelU = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [fRotationMatrixUser]
    // Deserialize array length for message field [fRotationMatrixUser]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.fRotationMatrixUser = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.fRotationMatrixUser[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [fActualAI]
    data.fActualAI = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [bActualSW]
    data.bActualSW = _arrayDeserializer.bool(buffer, bufferOffset, 3)
    // Deserialize message field [bActualSI]
    data.bActualSI = _arrayDeserializer.bool(buffer, bufferOffset, 2)
    // Deserialize message field [iActualAT]
    data.iActualAT = _arrayDeserializer.int8(buffer, bufferOffset, 2)
    // Deserialize message field [fTargetAO]
    data.fTargetAO = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [iTargetAT]
    data.iTargetAT = _arrayDeserializer.int8(buffer, bufferOffset, 2)
    // Deserialize message field [bActualES]
    data.bActualES = _arrayDeserializer.bool(buffer, bufferOffset, 2)
    // Deserialize message field [iActualED]
    data.iActualED = _arrayDeserializer.int8(buffer, bufferOffset, 2)
    // Deserialize message field [bActualER]
    data.bActualER = _arrayDeserializer.bool(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robot_state_str);
    object.rotation_matrix.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.modbus_state.forEach((val) => {
      length += ModbusState.getMessageSize(val);
    });
    object.fCurrentPosW.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.fRotationMatrixWorld.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.fCurrentPosU.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    object.fRotationMatrixUser.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 1484;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0473bf35fc3b2d88cf36052d2ba4677e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # state of robot
    #____________________________________________________________________________________________
    
    int32       robot_state             # 0 : STATE_INITIALIZING,   1 : STATE_STANDBY,   2 : STATE_MOVING,    3 : STATE_SAFE_OFF
                                        # 4 : STATE_TEACHING,       5 : STATE_SAFE_STOP, 6 : STATE_EMERGENCY_STOP,
                                        # 7 : STATE_EMERGENCY_STOP, 8 : STATE_HOMMING,   9 : STATE_RECOVERY,  10: STATE_SAFE_STOP2,     
                                        # 11: STATE_SAFE_OFF2,      12: STATE_RESERVED1, 13: STATE_RESERVED2, 14: STATE_RESERVED3,
                                        # 15: STATE_NOT_READY       16: STATE_LAST
    string      robot_state_str         # Convert robot_state id to string
    
    int8        actual_mode             # position control: 0, torque control: 1
    int8        actual_space            # joint space: 0, task space: 1
    
    float64[6]  current_posj            # current joint angle list [degree] 
    float64[6]  current_velj            # current joint velocity list []
    float64[6]  joint_abs               # Position Actual Value in ABS
    float64[6]  joint_err               # Joint Error
    float64[6]  target_posj             # target joint angle list [degree] 
    float64[6]  target_velj             # target joint velocity list []
    
    float64[6]  current_posx            # current task angle list []
    float64[6]  current_tool_posx       # current task tool angle list []
    float64[6]  current_velx            # current task velocity list []
    float64[6]  task_err                # Task Error
    float64[6]  target_velx             # target task velocity list []       
    float64[6]  target_posx             # target task position list []
      
    float64[6]  dynamic_tor             # dynamic torque
    float64[6]  actual_jts              # joint torque sensor
    float64[6]  actual_ejt              # external joint torque
    float64[6]  actual_ett              # external tool torque
    
    int8[6]     actual_bk               # brake status 
    float64[6]  actual_mc               # motor current 
    float64[6]  actual_mt               # motor temperature
    
    int8        solution_space          # Solution Space (0 ~ 7)
    float64     sync_time               # internal clock counter   
    int8[5]     actual_bt               # cockpit(robot button) info.
    std_msgs/Float64MultiArray[] rotation_matrix  # Rotation Matrix [3][3]
    
    
    int8[16]    ctrlbox_digital_input   # Digital Input in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)
    int8[16]    ctrlbox_digital_output  # Digital Output in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)
    
    int8[6]     flange_digital_input    # Digital Input in Flange(0 ~ 5 ; 0 : ON, 1 : OFF) x1 port : 0 ~ 2, x2 port : 3 ~ 5
    int8[6]     flange_digital_output   # Digital Output in Flange(0 ~ 5 ; 0 : ON, 1 : OFF)
    
    ModbusState[] modbus_state          # Custom msg for modbus state(refer to ModbusState.msg)
    int32 	    access_control          # 
    bool	    homming_completed       #
    bool	    tp_initialized          #
    int8	    mastering_need          #
    bool	    drl_stopped             #
    bool	    disconnected            #
    
    #____________________________________________________________________________________________
    # The following messages have been updated since version M2.50 or higher.
    #____________________________________________________________________________________________
    float64[6]  fActualW2B                               # world to base releation
    std_msgs/Float64MultiArray[] fCurrentPosW            # Wolrd position actual value [2][6] : (0: tool, 1: flange) [mm, degree] 
    float64[6]  fCurrentVelW                             # World velocity Actual Value [mm/sec, degree/sec]
    float64[6]  fWorldETT                                # External Task Force/Torque [N, Nm]
    float64[6]  fTargetPosW                              # World target Position [mm, degree]
    float64[6]  fTargetVelW                              # World target Velocity [mm/sec, degree/sec]
    std_msgs/Float64MultiArray[] fRotationMatrixWorld    # World rotation matrix [3][3]
    int8        iActualUCN                               # Actual user coord number ## 101 ~ 120
    int8        iParent                                  # Coordinate Reference(base : 0  world : 2)
    std_msgs/Float64MultiArray[] fCurrentPosU            # User position Actual Value [2][6] : (0:tool, 1:flange) [mm, degree]
    float64[6]  fCurrentVelU                             # User velocity Actual Value [mm/sec, degree/sec]
    float64[6]  fUserETT                                 # External Task Force/Torque [N, Nm]
    float64[6]  fTargetPosU                              # User target Position [mm, degree]
    float64[6]  fTargetVelU                              # User target Velocity [mm/sec, degree/sec]
    std_msgs/Float64MultiArray[] fRotationMatrixUser     # User rotation matrix [3][3] 
    float64[6]  fActualAI                                # Analog input data ## Current mode : 0~20.0[mA] , Voltage mode : 0~10.0[V]
    bool[3]     bActualSW                                # Switch input data[3] 
    bool[2]     bActualSI                                # Safety input data[2]
    int8[2]     iActualAT                                # Analog input type[2]  index = channel, type: current(0), voltage(1)  
    float64[2]    fTargetAO                              # Analog output data ## Current mode : 0~20.0[mA] , Voltage mode : 0~10.0[V]
    int8[2]     iTargetAT                                # Analog output type[2] index = channel, type: current(0), voltage(1)
    bool[2]     bActualES                                # Encorder strove signal
    int8[2]     iActualED                                # Encorder raw data   
    bool[2]     bActualER                                # Encorder reset signal
    
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
    ================================================================================
    MSG: dsr_msgs/ModbusState
    #____________________________________________________________________________________________
    #Custom msg for RobotState.msg -- MAX_SIZE = 100
    #____________________________________________________________________________________________
    
    string  modbus_symbol    # Modbus Signal Name
    int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~ 65535)
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotState(null);
    if (msg.robot_state !== undefined) {
      resolved.robot_state = msg.robot_state;
    }
    else {
      resolved.robot_state = 0
    }

    if (msg.robot_state_str !== undefined) {
      resolved.robot_state_str = msg.robot_state_str;
    }
    else {
      resolved.robot_state_str = ''
    }

    if (msg.actual_mode !== undefined) {
      resolved.actual_mode = msg.actual_mode;
    }
    else {
      resolved.actual_mode = 0
    }

    if (msg.actual_space !== undefined) {
      resolved.actual_space = msg.actual_space;
    }
    else {
      resolved.actual_space = 0
    }

    if (msg.current_posj !== undefined) {
      resolved.current_posj = msg.current_posj;
    }
    else {
      resolved.current_posj = new Array(6).fill(0)
    }

    if (msg.current_velj !== undefined) {
      resolved.current_velj = msg.current_velj;
    }
    else {
      resolved.current_velj = new Array(6).fill(0)
    }

    if (msg.joint_abs !== undefined) {
      resolved.joint_abs = msg.joint_abs;
    }
    else {
      resolved.joint_abs = new Array(6).fill(0)
    }

    if (msg.joint_err !== undefined) {
      resolved.joint_err = msg.joint_err;
    }
    else {
      resolved.joint_err = new Array(6).fill(0)
    }

    if (msg.target_posj !== undefined) {
      resolved.target_posj = msg.target_posj;
    }
    else {
      resolved.target_posj = new Array(6).fill(0)
    }

    if (msg.target_velj !== undefined) {
      resolved.target_velj = msg.target_velj;
    }
    else {
      resolved.target_velj = new Array(6).fill(0)
    }

    if (msg.current_posx !== undefined) {
      resolved.current_posx = msg.current_posx;
    }
    else {
      resolved.current_posx = new Array(6).fill(0)
    }

    if (msg.current_tool_posx !== undefined) {
      resolved.current_tool_posx = msg.current_tool_posx;
    }
    else {
      resolved.current_tool_posx = new Array(6).fill(0)
    }

    if (msg.current_velx !== undefined) {
      resolved.current_velx = msg.current_velx;
    }
    else {
      resolved.current_velx = new Array(6).fill(0)
    }

    if (msg.task_err !== undefined) {
      resolved.task_err = msg.task_err;
    }
    else {
      resolved.task_err = new Array(6).fill(0)
    }

    if (msg.target_velx !== undefined) {
      resolved.target_velx = msg.target_velx;
    }
    else {
      resolved.target_velx = new Array(6).fill(0)
    }

    if (msg.target_posx !== undefined) {
      resolved.target_posx = msg.target_posx;
    }
    else {
      resolved.target_posx = new Array(6).fill(0)
    }

    if (msg.dynamic_tor !== undefined) {
      resolved.dynamic_tor = msg.dynamic_tor;
    }
    else {
      resolved.dynamic_tor = new Array(6).fill(0)
    }

    if (msg.actual_jts !== undefined) {
      resolved.actual_jts = msg.actual_jts;
    }
    else {
      resolved.actual_jts = new Array(6).fill(0)
    }

    if (msg.actual_ejt !== undefined) {
      resolved.actual_ejt = msg.actual_ejt;
    }
    else {
      resolved.actual_ejt = new Array(6).fill(0)
    }

    if (msg.actual_ett !== undefined) {
      resolved.actual_ett = msg.actual_ett;
    }
    else {
      resolved.actual_ett = new Array(6).fill(0)
    }

    if (msg.actual_bk !== undefined) {
      resolved.actual_bk = msg.actual_bk;
    }
    else {
      resolved.actual_bk = new Array(6).fill(0)
    }

    if (msg.actual_mc !== undefined) {
      resolved.actual_mc = msg.actual_mc;
    }
    else {
      resolved.actual_mc = new Array(6).fill(0)
    }

    if (msg.actual_mt !== undefined) {
      resolved.actual_mt = msg.actual_mt;
    }
    else {
      resolved.actual_mt = new Array(6).fill(0)
    }

    if (msg.solution_space !== undefined) {
      resolved.solution_space = msg.solution_space;
    }
    else {
      resolved.solution_space = 0
    }

    if (msg.sync_time !== undefined) {
      resolved.sync_time = msg.sync_time;
    }
    else {
      resolved.sync_time = 0.0
    }

    if (msg.actual_bt !== undefined) {
      resolved.actual_bt = msg.actual_bt;
    }
    else {
      resolved.actual_bt = new Array(5).fill(0)
    }

    if (msg.rotation_matrix !== undefined) {
      resolved.rotation_matrix = new Array(msg.rotation_matrix.length);
      for (let i = 0; i < resolved.rotation_matrix.length; ++i) {
        resolved.rotation_matrix[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.rotation_matrix[i]);
      }
    }
    else {
      resolved.rotation_matrix = []
    }

    if (msg.ctrlbox_digital_input !== undefined) {
      resolved.ctrlbox_digital_input = msg.ctrlbox_digital_input;
    }
    else {
      resolved.ctrlbox_digital_input = new Array(16).fill(0)
    }

    if (msg.ctrlbox_digital_output !== undefined) {
      resolved.ctrlbox_digital_output = msg.ctrlbox_digital_output;
    }
    else {
      resolved.ctrlbox_digital_output = new Array(16).fill(0)
    }

    if (msg.flange_digital_input !== undefined) {
      resolved.flange_digital_input = msg.flange_digital_input;
    }
    else {
      resolved.flange_digital_input = new Array(6).fill(0)
    }

    if (msg.flange_digital_output !== undefined) {
      resolved.flange_digital_output = msg.flange_digital_output;
    }
    else {
      resolved.flange_digital_output = new Array(6).fill(0)
    }

    if (msg.modbus_state !== undefined) {
      resolved.modbus_state = new Array(msg.modbus_state.length);
      for (let i = 0; i < resolved.modbus_state.length; ++i) {
        resolved.modbus_state[i] = ModbusState.Resolve(msg.modbus_state[i]);
      }
    }
    else {
      resolved.modbus_state = []
    }

    if (msg.access_control !== undefined) {
      resolved.access_control = msg.access_control;
    }
    else {
      resolved.access_control = 0
    }

    if (msg.homming_completed !== undefined) {
      resolved.homming_completed = msg.homming_completed;
    }
    else {
      resolved.homming_completed = false
    }

    if (msg.tp_initialized !== undefined) {
      resolved.tp_initialized = msg.tp_initialized;
    }
    else {
      resolved.tp_initialized = false
    }

    if (msg.mastering_need !== undefined) {
      resolved.mastering_need = msg.mastering_need;
    }
    else {
      resolved.mastering_need = 0
    }

    if (msg.drl_stopped !== undefined) {
      resolved.drl_stopped = msg.drl_stopped;
    }
    else {
      resolved.drl_stopped = false
    }

    if (msg.disconnected !== undefined) {
      resolved.disconnected = msg.disconnected;
    }
    else {
      resolved.disconnected = false
    }

    if (msg.fActualW2B !== undefined) {
      resolved.fActualW2B = msg.fActualW2B;
    }
    else {
      resolved.fActualW2B = new Array(6).fill(0)
    }

    if (msg.fCurrentPosW !== undefined) {
      resolved.fCurrentPosW = new Array(msg.fCurrentPosW.length);
      for (let i = 0; i < resolved.fCurrentPosW.length; ++i) {
        resolved.fCurrentPosW[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.fCurrentPosW[i]);
      }
    }
    else {
      resolved.fCurrentPosW = []
    }

    if (msg.fCurrentVelW !== undefined) {
      resolved.fCurrentVelW = msg.fCurrentVelW;
    }
    else {
      resolved.fCurrentVelW = new Array(6).fill(0)
    }

    if (msg.fWorldETT !== undefined) {
      resolved.fWorldETT = msg.fWorldETT;
    }
    else {
      resolved.fWorldETT = new Array(6).fill(0)
    }

    if (msg.fTargetPosW !== undefined) {
      resolved.fTargetPosW = msg.fTargetPosW;
    }
    else {
      resolved.fTargetPosW = new Array(6).fill(0)
    }

    if (msg.fTargetVelW !== undefined) {
      resolved.fTargetVelW = msg.fTargetVelW;
    }
    else {
      resolved.fTargetVelW = new Array(6).fill(0)
    }

    if (msg.fRotationMatrixWorld !== undefined) {
      resolved.fRotationMatrixWorld = new Array(msg.fRotationMatrixWorld.length);
      for (let i = 0; i < resolved.fRotationMatrixWorld.length; ++i) {
        resolved.fRotationMatrixWorld[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.fRotationMatrixWorld[i]);
      }
    }
    else {
      resolved.fRotationMatrixWorld = []
    }

    if (msg.iActualUCN !== undefined) {
      resolved.iActualUCN = msg.iActualUCN;
    }
    else {
      resolved.iActualUCN = 0
    }

    if (msg.iParent !== undefined) {
      resolved.iParent = msg.iParent;
    }
    else {
      resolved.iParent = 0
    }

    if (msg.fCurrentPosU !== undefined) {
      resolved.fCurrentPosU = new Array(msg.fCurrentPosU.length);
      for (let i = 0; i < resolved.fCurrentPosU.length; ++i) {
        resolved.fCurrentPosU[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.fCurrentPosU[i]);
      }
    }
    else {
      resolved.fCurrentPosU = []
    }

    if (msg.fCurrentVelU !== undefined) {
      resolved.fCurrentVelU = msg.fCurrentVelU;
    }
    else {
      resolved.fCurrentVelU = new Array(6).fill(0)
    }

    if (msg.fUserETT !== undefined) {
      resolved.fUserETT = msg.fUserETT;
    }
    else {
      resolved.fUserETT = new Array(6).fill(0)
    }

    if (msg.fTargetPosU !== undefined) {
      resolved.fTargetPosU = msg.fTargetPosU;
    }
    else {
      resolved.fTargetPosU = new Array(6).fill(0)
    }

    if (msg.fTargetVelU !== undefined) {
      resolved.fTargetVelU = msg.fTargetVelU;
    }
    else {
      resolved.fTargetVelU = new Array(6).fill(0)
    }

    if (msg.fRotationMatrixUser !== undefined) {
      resolved.fRotationMatrixUser = new Array(msg.fRotationMatrixUser.length);
      for (let i = 0; i < resolved.fRotationMatrixUser.length; ++i) {
        resolved.fRotationMatrixUser[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.fRotationMatrixUser[i]);
      }
    }
    else {
      resolved.fRotationMatrixUser = []
    }

    if (msg.fActualAI !== undefined) {
      resolved.fActualAI = msg.fActualAI;
    }
    else {
      resolved.fActualAI = new Array(6).fill(0)
    }

    if (msg.bActualSW !== undefined) {
      resolved.bActualSW = msg.bActualSW;
    }
    else {
      resolved.bActualSW = new Array(3).fill(0)
    }

    if (msg.bActualSI !== undefined) {
      resolved.bActualSI = msg.bActualSI;
    }
    else {
      resolved.bActualSI = new Array(2).fill(0)
    }

    if (msg.iActualAT !== undefined) {
      resolved.iActualAT = msg.iActualAT;
    }
    else {
      resolved.iActualAT = new Array(2).fill(0)
    }

    if (msg.fTargetAO !== undefined) {
      resolved.fTargetAO = msg.fTargetAO;
    }
    else {
      resolved.fTargetAO = new Array(2).fill(0)
    }

    if (msg.iTargetAT !== undefined) {
      resolved.iTargetAT = msg.iTargetAT;
    }
    else {
      resolved.iTargetAT = new Array(2).fill(0)
    }

    if (msg.bActualES !== undefined) {
      resolved.bActualES = msg.bActualES;
    }
    else {
      resolved.bActualES = new Array(2).fill(0)
    }

    if (msg.iActualED !== undefined) {
      resolved.iActualED = msg.iActualED;
    }
    else {
      resolved.iActualED = new Array(2).fill(0)
    }

    if (msg.bActualER !== undefined) {
      resolved.bActualER = msg.bActualER;
    }
    else {
      resolved.bActualER = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = RobotState;
