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

class DrlStartRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robotSystem = null;
      this.code = null;
    }
    else {
      if (initObj.hasOwnProperty('robotSystem')) {
        this.robotSystem = initObj.robotSystem
      }
      else {
        this.robotSystem = 0;
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DrlStartRequest
    // Serialize message field [robotSystem]
    bufferOffset = _serializer.int8(obj.robotSystem, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.string(obj.code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DrlStartRequest
    let len;
    let data = new DrlStartRequest(null);
    // Deserialize message field [robotSystem]
    data.robotSystem = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.code);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/DrlStartRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '72b9065a09fde123af1be0fe946576d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # drl_script_run  
    # This is a service to execute a program configured in the DRL language in the robot controller.
    #____________________________________________________________________________________________
    
    int8 robotSystem    # Robot System Mode 0 : Real, 1 : virtual
    string  code        # drl code       
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DrlStartRequest(null);
    if (msg.robotSystem !== undefined) {
      resolved.robotSystem = msg.robotSystem;
    }
    else {
      resolved.robotSystem = 0
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = ''
    }

    return resolved;
    }
};

class DrlStartResponse {
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
    // Serializes a message object of type DrlStartResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DrlStartResponse
    let len;
    let data = new DrlStartResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/DrlStartResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DrlStartResponse(null);
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
  Request: DrlStartRequest,
  Response: DrlStartResponse,
  md5sum() { return '141a24be8b7e223a4678cb5eeb926bd6'; },
  datatype() { return 'dsr_msgs/DrlStart'; }
};
