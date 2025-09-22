// Auto-generated. Do not edit!

// (in-package dsr_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class WriteDataRTRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.external_force_torque = null;
      this.external_digital_input = null;
      this.external_digital_output = null;
      this.external_analog_input = null;
      this.external_analog_output = null;
    }
    else {
      if (initObj.hasOwnProperty('external_force_torque')) {
        this.external_force_torque = initObj.external_force_torque
      }
      else {
        this.external_force_torque = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('external_digital_input')) {
        this.external_digital_input = initObj.external_digital_input
      }
      else {
        this.external_digital_input = 0;
      }
      if (initObj.hasOwnProperty('external_digital_output')) {
        this.external_digital_output = initObj.external_digital_output
      }
      else {
        this.external_digital_output = 0;
      }
      if (initObj.hasOwnProperty('external_analog_input')) {
        this.external_analog_input = initObj.external_analog_input
      }
      else {
        this.external_analog_input = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('external_analog_output')) {
        this.external_analog_output = initObj.external_analog_output
      }
      else {
        this.external_analog_output = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WriteDataRTRequest
    // Check that the constant length array field [external_force_torque] has the right length
    if (obj.external_force_torque.length !== 6) {
      throw new Error('Unable to serialize array field external_force_torque - length must be 6')
    }
    // Serialize message field [external_force_torque]
    bufferOffset = _arraySerializer.float64(obj.external_force_torque, buffer, bufferOffset, 6);
    // Serialize message field [external_digital_input]
    bufferOffset = _serializer.int32(obj.external_digital_input, buffer, bufferOffset);
    // Serialize message field [external_digital_output]
    bufferOffset = _serializer.int32(obj.external_digital_output, buffer, bufferOffset);
    // Check that the constant length array field [external_analog_input] has the right length
    if (obj.external_analog_input.length !== 6) {
      throw new Error('Unable to serialize array field external_analog_input - length must be 6')
    }
    // Serialize message field [external_analog_input]
    bufferOffset = _arraySerializer.float64(obj.external_analog_input, buffer, bufferOffset, 6);
    // Check that the constant length array field [external_analog_output] has the right length
    if (obj.external_analog_output.length !== 6) {
      throw new Error('Unable to serialize array field external_analog_output - length must be 6')
    }
    // Serialize message field [external_analog_output]
    bufferOffset = _arraySerializer.float64(obj.external_analog_output, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WriteDataRTRequest
    let len;
    let data = new WriteDataRTRequest(null);
    // Deserialize message field [external_force_torque]
    data.external_force_torque = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [external_digital_input]
    data.external_digital_input = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [external_digital_output]
    data.external_digital_output = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [external_analog_input]
    data.external_analog_input = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [external_analog_output]
    data.external_analog_output = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 152;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/WriteDataRTRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0677aa4f576b1a48e5f18c66c91f29d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # write_data_rt
    #____________________________________________________________________________________________
    float64[6] external_force_torque
    int32      external_digital_input
    int32      external_digital_output
    float64[6] external_analog_input
    float64[6] external_analog_output
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WriteDataRTRequest(null);
    if (msg.external_force_torque !== undefined) {
      resolved.external_force_torque = msg.external_force_torque;
    }
    else {
      resolved.external_force_torque = new Array(6).fill(0)
    }

    if (msg.external_digital_input !== undefined) {
      resolved.external_digital_input = msg.external_digital_input;
    }
    else {
      resolved.external_digital_input = 0
    }

    if (msg.external_digital_output !== undefined) {
      resolved.external_digital_output = msg.external_digital_output;
    }
    else {
      resolved.external_digital_output = 0
    }

    if (msg.external_analog_input !== undefined) {
      resolved.external_analog_input = msg.external_analog_input;
    }
    else {
      resolved.external_analog_input = new Array(6).fill(0)
    }

    if (msg.external_analog_output !== undefined) {
      resolved.external_analog_output = msg.external_analog_output;
    }
    else {
      resolved.external_analog_output = new Array(6).fill(0)
    }

    return resolved;
    }
};

class WriteDataRTResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WriteDataRTResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WriteDataRTResponse
    let len;
    let data = new WriteDataRTResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/WriteDataRTResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WriteDataRTResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: WriteDataRTRequest,
  Response: WriteDataRTResponse,
  md5sum() { return '57fbcf1b749a9e589d5aa77f87216c0e'; },
  datatype() { return 'dsr_msgs/WriteDataRT'; }
};
