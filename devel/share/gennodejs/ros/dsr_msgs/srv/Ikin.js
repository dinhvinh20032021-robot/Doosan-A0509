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

class IkinRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.sol_space = null;
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('sol_space')) {
        this.sol_space = initObj.sol_space
      }
      else {
        this.sol_space = 0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IkinRequest
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 6) {
      throw new Error('Unable to serialize array field pos - length must be 6')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 6);
    // Serialize message field [sol_space]
    bufferOffset = _serializer.int8(obj.sol_space, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkinRequest
    let len;
    let data = new IkinRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [sol_space]
    data.sol_space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 50;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IkinRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ea6410df00826de29fa23b7537cf261';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # ikin  
    #____________________________________________________________________________________________
    
    float64[6] pos               # task pos(posx)  
    int8       sol_space         # solution space : 0 ~ 7
    int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkinRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(6).fill(0)
    }

    if (msg.sol_space !== undefined) {
      resolved.sol_space = msg.sol_space;
    }
    else {
      resolved.sol_space = 0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    return resolved;
    }
};

class IkinResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.conv_posj = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('conv_posj')) {
        this.conv_posj = initObj.conv_posj
      }
      else {
        this.conv_posj = new Array(6).fill(0);
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
    // Serializes a message object of type IkinResponse
    // Check that the constant length array field [conv_posj] has the right length
    if (obj.conv_posj.length !== 6) {
      throw new Error('Unable to serialize array field conv_posj - length must be 6')
    }
    // Serialize message field [conv_posj]
    bufferOffset = _arraySerializer.float64(obj.conv_posj, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkinResponse
    let len;
    let data = new IkinResponse(null);
    // Deserialize message field [conv_posj]
    data.conv_posj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IkinResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ba9fe60ca6f30087b9a59ed393c0603';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] conv_posj         # joint pos(posj)  
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkinResponse(null);
    if (msg.conv_posj !== undefined) {
      resolved.conv_posj = msg.conv_posj;
    }
    else {
      resolved.conv_posj = new Array(6).fill(0)
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
  Request: IkinRequest,
  Response: IkinResponse,
  md5sum() { return 'fb475e378a19f7cef6d8ac54d5b6fb72'; },
  datatype() { return 'dsr_msgs/Ikin'; }
};
