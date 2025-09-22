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

class GetCurrentSolutionSpaceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentSolutionSpaceRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentSolutionSpaceRequest
    let len;
    let data = new GetCurrentSolutionSpaceRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentSolutionSpaceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_current_solution_space
    #____________________________________________________________________________________________
    # This service returns the current solution space value.
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentSolutionSpaceRequest(null);
    return resolved;
    }
};

class GetCurrentSolutionSpaceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sol_space = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('sol_space')) {
        this.sol_space = initObj.sol_space
      }
      else {
        this.sol_space = 0;
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
    // Serializes a message object of type GetCurrentSolutionSpaceResponse
    // Serialize message field [sol_space]
    bufferOffset = _serializer.int8(obj.sol_space, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentSolutionSpaceResponse
    let len;
    let data = new GetCurrentSolutionSpaceResponse(null);
    // Deserialize message field [sol_space]
    data.sol_space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentSolutionSpaceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bacee50609041091b8cb888ffb4bd562';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8        sol_space         # solution space : 0 ~ 7
    bool        success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentSolutionSpaceResponse(null);
    if (msg.sol_space !== undefined) {
      resolved.sol_space = msg.sol_space;
    }
    else {
      resolved.sol_space = 0
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
  Request: GetCurrentSolutionSpaceRequest,
  Response: GetCurrentSolutionSpaceResponse,
  md5sum() { return 'bacee50609041091b8cb888ffb4bd562'; },
  datatype() { return 'dsr_msgs/GetCurrentSolutionSpace'; }
};
