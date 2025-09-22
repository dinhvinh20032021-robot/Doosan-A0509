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

class SetSafetyModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.safety_mode = null;
      this.safety_mode_event = null;
    }
    else {
      if (initObj.hasOwnProperty('safety_mode')) {
        this.safety_mode = initObj.safety_mode
      }
      else {
        this.safety_mode = 0;
      }
      if (initObj.hasOwnProperty('safety_mode_event')) {
        this.safety_mode_event = initObj.safety_mode_event
      }
      else {
        this.safety_mode_event = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSafetyModeRequest
    // Serialize message field [safety_mode]
    bufferOffset = _serializer.int8(obj.safety_mode, buffer, bufferOffset);
    // Serialize message field [safety_mode_event]
    bufferOffset = _serializer.int8(obj.safety_mode_event, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafetyModeRequest
    let len;
    let data = new SetSafetyModeRequest(null);
    // Deserialize message field [safety_mode]
    data.safety_mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [safety_mode_event]
    data.safety_mode_event = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetSafetyModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6504f596b701c2d5dab07a99c9b7f945';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_safety_mode
    #____________________________________________________________________________________________
    
    int8 safety_mode  
    int8 safety_mode_event
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSafetyModeRequest(null);
    if (msg.safety_mode !== undefined) {
      resolved.safety_mode = msg.safety_mode;
    }
    else {
      resolved.safety_mode = 0
    }

    if (msg.safety_mode_event !== undefined) {
      resolved.safety_mode_event = msg.safety_mode_event;
    }
    else {
      resolved.safety_mode_event = 0
    }

    return resolved;
    }
};

class SetSafetyModeResponse {
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
    // Serializes a message object of type SetSafetyModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafetyModeResponse
    let len;
    let data = new SetSafetyModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetSafetyModeResponse';
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
    const resolved = new SetSafetyModeResponse(null);
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
  Request: SetSafetyModeRequest,
  Response: SetSafetyModeResponse,
  md5sum() { return '82c2323d83064354d4a93feb6b800608'; },
  datatype() { return 'dsr_msgs/SetSafetyMode'; }
};
