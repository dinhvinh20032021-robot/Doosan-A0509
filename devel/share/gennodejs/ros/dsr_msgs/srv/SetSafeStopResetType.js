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

class SetSafeStopResetTypeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reset_type = null;
    }
    else {
      if (initObj.hasOwnProperty('reset_type')) {
        this.reset_type = initObj.reset_type
      }
      else {
        this.reset_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSafeStopResetTypeRequest
    // Serialize message field [reset_type]
    bufferOffset = _serializer.int8(obj.reset_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafeStopResetTypeRequest
    let len;
    let data = new SetSafeStopResetTypeRequest(null);
    // Deserialize message field [reset_type]
    data.reset_type = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetSafeStopResetTypeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc984cc000f8da662666d043b2ed22bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_safe_stop_reset_type
    #____________________________________________________________________________________________
    
    int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSafeStopResetTypeRequest(null);
    if (msg.reset_type !== undefined) {
      resolved.reset_type = msg.reset_type;
    }
    else {
      resolved.reset_type = 0
    }

    return resolved;
    }
};

class SetSafeStopResetTypeResponse {
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
    // Serializes a message object of type SetSafeStopResetTypeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafeStopResetTypeResponse
    let len;
    let data = new SetSafeStopResetTypeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetSafeStopResetTypeResponse';
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
    const resolved = new SetSafeStopResetTypeResponse(null);
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
  Request: SetSafeStopResetTypeRequest,
  Response: SetSafeStopResetTypeResponse,
  md5sum() { return '956a2bdbbc2e1317a58568aa311b17b7'; },
  datatype() { return 'dsr_msgs/SetSafeStopResetType'; }
};
