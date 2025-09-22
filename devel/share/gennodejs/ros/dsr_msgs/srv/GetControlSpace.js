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

class GetControlSpaceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControlSpaceRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControlSpaceRequest
    let len;
    let data = new GetControlSpaceRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetControlSpaceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_control_space()  
    #____________________________________________________________________________________________
    # This service returns the current control space.
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControlSpaceRequest(null);
    return resolved;
    }
};

class GetControlSpaceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.space = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('space')) {
        this.space = initObj.space
      }
      else {
        this.space = 0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControlSpaceResponse
    // Serialize message field [space]
    bufferOffset = _serializer.int8(obj.space, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControlSpaceResponse
    let len;
    let data = new GetControlSpaceResponse(null);
    // Deserialize message field [space]
    data.space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetControlSpaceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a5780af2202e44900c27ba310bfe10c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8    space        # Control mode : Joint space control(1), Task space control(2)
    bool    success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControlSpaceResponse(null);
    if (msg.space !== undefined) {
      resolved.space = msg.space;
    }
    else {
      resolved.space = 0
    }

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
  Request: GetControlSpaceRequest,
  Response: GetControlSpaceResponse,
  md5sum() { return '1a5780af2202e44900c27ba310bfe10c'; },
  datatype() { return 'dsr_msgs/GetControlSpace'; }
};
