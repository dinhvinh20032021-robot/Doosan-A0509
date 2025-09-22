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

class TransRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.delta = null;
      this.ref = null;
      this.ref_out = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('delta')) {
        this.delta = initObj.delta
      }
      else {
        this.delta = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('ref_out')) {
        this.ref_out = initObj.ref_out
      }
      else {
        this.ref_out = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransRequest
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 6) {
      throw new Error('Unable to serialize array field pos - length must be 6')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 6);
    // Check that the constant length array field [delta] has the right length
    if (obj.delta.length !== 6) {
      throw new Error('Unable to serialize array field delta - length must be 6')
    }
    // Serialize message field [delta]
    bufferOffset = _arraySerializer.float64(obj.delta, buffer, bufferOffset, 6);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [ref_out]
    bufferOffset = _serializer.int8(obj.ref_out, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransRequest
    let len;
    let data = new TransRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [delta]
    data.delta = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref_out]
    data.ref_out = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 98;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/TransRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7ea759998e0dd768f4e6a42af908529b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # trans  
    #____________________________________________________________________________________________
    
    float64[6] pos               # task pos(posx)  
    float64[6] delta             # delta (posx)  
    int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                 # <DR_WORLD is only available in M2.40 or later> 
    int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)
                                 # <ref_out is only available in M2.40 or later>
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(6).fill(0)
    }

    if (msg.delta !== undefined) {
      resolved.delta = msg.delta;
    }
    else {
      resolved.delta = new Array(6).fill(0)
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.ref_out !== undefined) {
      resolved.ref_out = msg.ref_out;
    }
    else {
      resolved.ref_out = 0
    }

    return resolved;
    }
};

class TransResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trans_pos = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('trans_pos')) {
        this.trans_pos = initObj.trans_pos
      }
      else {
        this.trans_pos = new Array(6).fill(0);
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
    // Serializes a message object of type TransResponse
    // Check that the constant length array field [trans_pos] has the right length
    if (obj.trans_pos.length !== 6) {
      throw new Error('Unable to serialize array field trans_pos - length must be 6')
    }
    // Serialize message field [trans_pos]
    bufferOffset = _arraySerializer.float64(obj.trans_pos, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransResponse
    let len;
    let data = new TransResponse(null);
    // Deserialize message field [trans_pos]
    data.trans_pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/TransResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f4be8aedbb129060dd973de94cc35bcc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] trans_pos         # trans pos(posx) 
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransResponse(null);
    if (msg.trans_pos !== undefined) {
      resolved.trans_pos = msg.trans_pos;
    }
    else {
      resolved.trans_pos = new Array(6).fill(0)
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
  Request: TransRequest,
  Response: TransResponse,
  md5sum() { return '2cc8472f65a9eb8b986e1489e30025bf'; },
  datatype() { return 'dsr_msgs/Trans'; }
};
