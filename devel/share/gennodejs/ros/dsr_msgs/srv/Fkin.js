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

class FkinRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.ref = null;
    }
    else {
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
    // Serializes a message object of type FkinRequest
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
    //deserializes a message object of type FkinRequest
    let len;
    let data = new FkinRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/FkinRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '01a7d76771e91045d7b0913667745cf9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # fkin  
    #____________________________________________________________________________________________
    
    float64[6] pos               # joint pos(posj)  
    int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FkinRequest(null);
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

class FkinResponse {
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
    // Serializes a message object of type FkinResponse
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
    //deserializes a message object of type FkinResponse
    let len;
    let data = new FkinResponse(null);
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
    return 'dsr_msgs/FkinResponse';
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
    const resolved = new FkinResponse(null);
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
  Request: FkinRequest,
  Response: FkinResponse,
  md5sum() { return '26c14bd0ea35ff7d87f4f121a2ae40ac'; },
  datatype() { return 'dsr_msgs/Fkin'; }
};
