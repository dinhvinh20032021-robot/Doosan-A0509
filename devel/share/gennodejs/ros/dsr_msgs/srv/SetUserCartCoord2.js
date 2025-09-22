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

class SetUserCartCoord2Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x1 = null;
      this.x2 = null;
      this.x3 = null;
      this.pos = null;
      this.ref = null;
    }
    else {
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
    // Serializes a message object of type SetUserCartCoord2Request
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
    //deserializes a message object of type SetUserCartCoord2Request
    let len;
    let data = new SetUserCartCoord2Request(null);
    // Deserialize message field [x1]
    data.x1 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x2]
    data.x2 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x3]
    data.x3 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 193;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetUserCartCoord2Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b9f3dcf6a65143791504d0e9c825e501';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_user_cart_coord(x1, x2, x3, pos, ref)
    #____________________________________________________________________________________________
    
    float64[6] x1                 # task pos(posx)  
    float64[6] x2                 # task pos(posx)  
    float64[6] x3                 # task pos(posx)
    float64[6] pos                # pos(posx)
    int8       ref                # DR_BASE(0), DR_WORLD(2)
                                  # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUserCartCoord2Request(null);
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

class SetUserCartCoord2Response {
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
    // Serializes a message object of type SetUserCartCoord2Response
    // Serialize message field [id]
    bufferOffset = _serializer.int8(obj.id, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUserCartCoord2Response
    let len;
    let data = new SetUserCartCoord2Response(null);
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
    return 'dsr_msgs/SetUserCartCoord2Response';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5845c23151db63ce4fa936d927189f8d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8    id                    # set user coord (101~200) or fail(-1) 
    bool        success   
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetUserCartCoord2Response(null);
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
  Request: SetUserCartCoord2Request,
  Response: SetUserCartCoord2Response,
  md5sum() { return '42d11c6c94c1dfc41a37882dbd6da758'; },
  datatype() { return 'dsr_msgs/SetUserCartCoord2'; }
};
