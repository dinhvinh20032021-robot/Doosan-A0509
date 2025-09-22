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

class GetSolutionSpaceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSolutionSpaceRequest
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 6) {
      throw new Error('Unable to serialize array field pos - length must be 6')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSolutionSpaceRequest
    let len;
    let data = new GetSolutionSpaceRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetSolutionSpaceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '537431324117c3d1847d70e057990155';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_solution_space(pos)  
    #____________________________________________________________________________________________
    # This service obtains the solution space value.
    
    float64[6] pos               # joint angle list [degree] 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSolutionSpaceRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(6).fill(0)
    }

    return resolved;
    }
};

class GetSolutionSpaceResponse {
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
    // Serializes a message object of type GetSolutionSpaceResponse
    // Serialize message field [sol_space]
    bufferOffset = _serializer.int8(obj.sol_space, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSolutionSpaceResponse
    let len;
    let data = new GetSolutionSpaceResponse(null);
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
    return 'dsr_msgs/GetSolutionSpaceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bacee50609041091b8cb888ffb4bd562';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8       sol_space         # solution space : 0 ~ 7
    bool       success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSolutionSpaceResponse(null);
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
  Request: GetSolutionSpaceRequest,
  Response: GetSolutionSpaceResponse,
  md5sum() { return '27abbe19a7d41c61a3226076e3320ebf'; },
  datatype() { return 'dsr_msgs/GetSolutionSpace'; }
};
