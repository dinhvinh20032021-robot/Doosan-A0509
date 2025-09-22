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

class SetUserCartCoord3Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u1 = null;
      this.v1 = null;
      this.pos = null;
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('u1')) {
        this.u1 = initObj.u1
      }
      else {
        this.u1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('v1')) {
        this.v1 = initObj.v1
      }
      else {
        this.v1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(6).fill(0);
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
    // Serializes a message object of type SetUserCartCoord3Request
    // Check that the constant length array field [u1] has the right length
    if (obj.u1.length !== 3) {
      throw new Error('Unable to serialize array field u1 - length must be 3')
    }
    // Serialize message field [u1]
    bufferOffset = _arraySerializer.float64(obj.u1, buffer, bufferOffset, 3);
    // Check that the constant length array field [v1] has the right length
    if (obj.v1.length !== 3) {
      throw new Error('Unable to serialize array field v1 - length must be 3')
    }
    // Serialize message field [v1]
    bufferOffset = _arraySerializer.float64(obj.v1, buffer, bufferOffset, 3);
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 6) {
      throw new Error('Unable to serialize array field pos - length must be 6')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 6);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUserCartCoord3Request
    let len;
    let data = new SetUserCartCoord3Request(null);
    // Deserialize message field [u1]
    data.u1 = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [v1]
    data.v1 = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 97;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetUserCartCoord3Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c5f03fe1f944ac766f5096fdf844e8a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_user_cart_coord(u1, v1, pos, ref) 
    #____________________________________________________________________________________________
    
    float64[3] u1                # X-axis unit vector  
    float64[3] v1                # Y-axis unit vector 
    float64[6] pos               # task pos(posx) 
    int8       ref               # DR_BASE(0), DR_WORLD(2)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUserCartCoord3Request(null);
    if (msg.u1 !== undefined) {
      resolved.u1 = msg.u1;
    }
    else {
      resolved.u1 = new Array(3).fill(0)
    }

    if (msg.v1 !== undefined) {
      resolved.v1 = msg.v1;
    }
    else {
      resolved.v1 = new Array(3).fill(0)
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(6).fill(0)
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

class SetUserCartCoord3Response {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
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
    // Serializes a message object of type SetUserCartCoord3Response
    // Serialize message field [id]
    bufferOffset = _serializer.int8(obj.id, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUserCartCoord3Response
    let len;
    let data = new SetUserCartCoord3Response(null);
    // Deserialize message field [id]
    data.id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetUserCartCoord3Response';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5845c23151db63ce4fa936d927189f8d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8    id                   # set user coord (101~120) or fail(-1) 
    bool        success   
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUserCartCoord3Response(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
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
  Request: SetUserCartCoord3Request,
  Response: SetUserCartCoord3Response,
  md5sum() { return 'c6fddb22deca3c301bea04b53df0d0a5'; },
  datatype() { return 'dsr_msgs/SetUserCartCoord3'; }
};
