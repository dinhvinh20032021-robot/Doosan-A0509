// Auto-generated. Do not edit!

// (in-package dsr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ModbusState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.modbus_symbol = null;
      this.modbus_value = null;
    }
    else {
      if (initObj.hasOwnProperty('modbus_symbol')) {
        this.modbus_symbol = initObj.modbus_symbol
      }
      else {
        this.modbus_symbol = '';
      }
      if (initObj.hasOwnProperty('modbus_value')) {
        this.modbus_value = initObj.modbus_value
      }
      else {
        this.modbus_value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModbusState
    // Serialize message field [modbus_symbol]
    bufferOffset = _serializer.string(obj.modbus_symbol, buffer, bufferOffset);
    // Serialize message field [modbus_value]
    bufferOffset = _serializer.int32(obj.modbus_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModbusState
    let len;
    let data = new ModbusState(null);
    // Deserialize message field [modbus_symbol]
    data.modbus_symbol = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [modbus_value]
    data.modbus_value = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.modbus_symbol);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/ModbusState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c941aa5cf7a235f21e3789f2d6e5305f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ModbusState(null);
    if (msg.modbus_symbol !== undefined) {
      resolved.modbus_symbol = msg.modbus_symbol;
    }
    else {
      resolved.modbus_symbol = ''
    }

    if (msg.modbus_value !== undefined) {
      resolved.modbus_value = msg.modbus_value;
    }
    else {
      resolved.modbus_value = 0
    }

    return resolved;
    }
};

module.exports = ModbusState;
