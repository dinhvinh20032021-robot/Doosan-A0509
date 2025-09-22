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

class DrlStopRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stop_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('stop_mode')) {
        this.stop_mode = initObj.stop_mode
      }
      else {
        this.stop_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DrlStopRequest
    // Serialize message field [stop_mode]
    bufferOffset = _serializer.int8(obj.stop_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DrlStopRequest
    let len;
    let data = new DrlStopRequest(null);
    // Deserialize message field [stop_mode]
    data.stop_mode = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/DrlStopRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0adf90a96bdab6d3a2cf8b6c4744e5bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # drl_script_stop  
    # STOP_TYPE_QUICK_STO = 0
    # STOP_TYPE_QUICK     = 1
    # STOP_TYPE_SLOW      = 2
    # STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  
    #____________________________________________________________________________________________
    
    int8    stop_mode       # <STOP_TYPE> stop_mode       
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DrlStopRequest(null);
    if (msg.stop_mode !== undefined) {
      resolved.stop_mode = msg.stop_mode;
    }
    else {
      resolved.stop_mode = 0
    }

    return resolved;
    }
};

class DrlStopResponse {
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
    // Serializes a message object of type DrlStopResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DrlStopResponse
    let len;
    let data = new DrlStopResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/DrlStopResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool    success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DrlStopResponse(null);
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
  Request: DrlStopRequest,
  Response: DrlStopResponse,
  md5sum() { return 'a7f8fadac11dc7732e1bdb5e5d128820'; },
  datatype() { return 'dsr_msgs/DrlStop'; }
};
