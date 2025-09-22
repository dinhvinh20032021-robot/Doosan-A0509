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

class IkinExRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.sol_space = null;
      this.ref = null;
      this.ref_pos_opt = null;
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
      if (initObj.hasOwnProperty('ref_pos_opt')) {
        this.ref_pos_opt = initObj.ref_pos_opt
      }
      else {
        this.ref_pos_opt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IkinExRequest
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
    // Serialize message field [ref_pos_opt]
    bufferOffset = _serializer.int8(obj.ref_pos_opt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkinExRequest
    let len;
    let data = new IkinExRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [sol_space]
    data.sol_space = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref_pos_opt]
    data.ref_pos_opt = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 51;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IkinExRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b0ee6e73013bde4af34cf17c72d4686';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # ikin_ex  
    #____________________________________________________________________________________________
    
    float64[6] pos               # task pos(posx)  
    int8       sol_space         # solution space : 0 ~ 7
    int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
    int8       ref_pos_opt	     #  
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkinExRequest(null);
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

    if (msg.ref_pos_opt !== undefined) {
      resolved.ref_pos_opt = msg.ref_pos_opt;
    }
    else {
      resolved.ref_pos_opt = 0
    }

    return resolved;
    }
};

class IkinExResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.conv_posj = null;
      this.status = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('conv_posj')) {
        this.conv_posj = initObj.conv_posj
      }
      else {
        this.conv_posj = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
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
    // Serializes a message object of type IkinExResponse
    // Check that the constant length array field [conv_posj] has the right length
    if (obj.conv_posj.length !== 6) {
      throw new Error('Unable to serialize array field conv_posj - length must be 6')
    }
    // Serialize message field [conv_posj]
    bufferOffset = _arraySerializer.float64(obj.conv_posj, buffer, bufferOffset, 6);
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkinExResponse
    let len;
    let data = new IkinExResponse(null);
    // Deserialize message field [conv_posj]
    data.conv_posj = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 50;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IkinExResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a262c9bd7c19457d248b6a5e85981bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] conv_posj         # joint pos(posj)  
    bool       status
    bool       success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkinExResponse(null);
    if (msg.conv_posj !== undefined) {
      resolved.conv_posj = msg.conv_posj;
    }
    else {
      resolved.conv_posj = new Array(6).fill(0)
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
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
  Request: IkinExRequest,
  Response: IkinExResponse,
  md5sum() { return '52c2edae66404c90e27c09703f4690c1'; },
  datatype() { return 'dsr_msgs/IkinEx'; }
};
