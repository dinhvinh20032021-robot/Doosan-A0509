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

class CalcCoordRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_pos_cnt = null;
      this.x1 = null;
      this.x2 = null;
      this.x3 = null;
      this.x4 = null;
      this.ref = null;
      this.mod = null;
    }
    else {
      if (initObj.hasOwnProperty('input_pos_cnt')) {
        this.input_pos_cnt = initObj.input_pos_cnt
      }
      else {
        this.input_pos_cnt = 0;
      }
      if (initObj.hasOwnProperty('x1')) {
        this.x1 = initObj.x1
      }
      else {
        this.x1 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('x2')) {
        this.x2 = initObj.x2
      }
      else {
        this.x2 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('x3')) {
        this.x3 = initObj.x3
      }
      else {
        this.x3 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('x4')) {
        this.x4 = initObj.x4
      }
      else {
        this.x4 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('mod')) {
        this.mod = initObj.mod
      }
      else {
        this.mod = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalcCoordRequest
    // Serialize message field [input_pos_cnt]
    bufferOffset = _serializer.int8(obj.input_pos_cnt, buffer, bufferOffset);
    // Check that the constant length array field [x1] has the right length
    if (obj.x1.length !== 6) {
      throw new Error('Unable to serialize array field x1 - length must be 6')
    }
    // Serialize message field [x1]
    bufferOffset = _arraySerializer.float64(obj.x1, buffer, bufferOffset, 6);
    // Check that the constant length array field [x2] has the right length
    if (obj.x2.length !== 6) {
      throw new Error('Unable to serialize array field x2 - length must be 6')
    }
    // Serialize message field [x2]
    bufferOffset = _arraySerializer.float64(obj.x2, buffer, bufferOffset, 6);
    // Check that the constant length array field [x3] has the right length
    if (obj.x3.length !== 6) {
      throw new Error('Unable to serialize array field x3 - length must be 6')
    }
    // Serialize message field [x3]
    bufferOffset = _arraySerializer.float64(obj.x3, buffer, bufferOffset, 6);
    // Check that the constant length array field [x4] has the right length
    if (obj.x4.length !== 6) {
      throw new Error('Unable to serialize array field x4 - length must be 6')
    }
    // Serialize message field [x4]
    bufferOffset = _arraySerializer.float64(obj.x4, buffer, bufferOffset, 6);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [mod]
    bufferOffset = _serializer.int8(obj.mod, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalcCoordRequest
    let len;
    let data = new CalcCoordRequest(null);
    // Deserialize message field [input_pos_cnt]
    data.input_pos_cnt = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [x1]
    data.x1 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x2]
    data.x2 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x3]
    data.x3 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x4]
    data.x4 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [mod]
    data.mod = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 195;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/CalcCoordRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f183627610e5714b756bba5cf0bcf435';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # calc_coord   
    #____________________________________________________________________________________________
    # This service is only available in M2.50 or later
    
    int8       input_pos_cnt     # input_pos_cnt
    float64[6] x1                # task pos(posx)  
    float64[6] x2                # task pos(posx)  
    float64[6] x3                # task pos(posx)
    float64[6] x4                # task pos(posx)
    int8       ref               # DR_BASE(0), DR_WORLD(2)
    int8       mod               # input mode(only valid when the number of input poses is 2)
                                 # 0: defining z-axis based on the current Tool-z direction
                                 # 1: defining z-axis based on the z direction of x1 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalcCoordRequest(null);
    if (msg.input_pos_cnt !== undefined) {
      resolved.input_pos_cnt = msg.input_pos_cnt;
    }
    else {
      resolved.input_pos_cnt = 0
    }

    if (msg.x1 !== undefined) {
      resolved.x1 = msg.x1;
    }
    else {
      resolved.x1 = new Array(6).fill(0)
    }

    if (msg.x2 !== undefined) {
      resolved.x2 = msg.x2;
    }
    else {
      resolved.x2 = new Array(6).fill(0)
    }

    if (msg.x3 !== undefined) {
      resolved.x3 = msg.x3;
    }
    else {
      resolved.x3 = new Array(6).fill(0)
    }

    if (msg.x4 !== undefined) {
      resolved.x4 = msg.x4;
    }
    else {
      resolved.x4 = new Array(6).fill(0)
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.mod !== undefined) {
      resolved.mod = msg.mod;
    }
    else {
      resolved.mod = 0
    }

    return resolved;
    }
};

class CalcCoordResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.conv_posx = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('conv_posx')) {
        this.conv_posx = initObj.conv_posx
      }
      else {
        this.conv_posx = new Array(6).fill(0);
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
    // Serializes a message object of type CalcCoordResponse
    // Check that the constant length array field [conv_posx] has the right length
    if (obj.conv_posx.length !== 6) {
      throw new Error('Unable to serialize array field conv_posx - length must be 6')
    }
    // Serialize message field [conv_posx]
    bufferOffset = _arraySerializer.float64(obj.conv_posx, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalcCoordResponse
    let len;
    let data = new CalcCoordResponse(null);
    // Deserialize message field [conv_posx]
    data.conv_posx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/CalcCoordResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '904a2ae76f71ef4190bde0122093ec1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] conv_posx         # task pos(posx) 
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalcCoordResponse(null);
    if (msg.conv_posx !== undefined) {
      resolved.conv_posx = msg.conv_posx;
    }
    else {
      resolved.conv_posx = new Array(6).fill(0)
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
  Request: CalcCoordRequest,
  Response: CalcCoordResponse,
  md5sum() { return '2001e63c6ceb43ba2ea8431cc72302b9'; },
  datatype() { return 'dsr_msgs/CalcCoord'; }
};
