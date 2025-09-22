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

class CoordTransformRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_in = null;
      this.ref_in = null;
      this.ref_out = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_in')) {
        this.pos_in = initObj.pos_in
      }
      else {
        this.pos_in = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('ref_in')) {
        this.ref_in = initObj.ref_in
      }
      else {
        this.ref_in = 0;
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
    // Serializes a message object of type CoordTransformRequest
    // Check that the constant length array field [pos_in] has the right length
    if (obj.pos_in.length !== 6) {
      throw new Error('Unable to serialize array field pos_in - length must be 6')
    }
    // Serialize message field [pos_in]
    bufferOffset = _arraySerializer.float64(obj.pos_in, buffer, bufferOffset, 6);
    // Serialize message field [ref_in]
    bufferOffset = _serializer.int8(obj.ref_in, buffer, bufferOffset);
    // Serialize message field [ref_out]
    bufferOffset = _serializer.int8(obj.ref_out, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CoordTransformRequest
    let len;
    let data = new CoordTransformRequest(null);
    // Deserialize message field [pos_in]
    data.pos_in = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref_in]
    data.ref_in = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref_out]
    data.ref_out = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 50;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/CoordTransformRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c009b69dce33d5ca178fda3328c53716';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # coord_transform   
    #____________________________________________________________________________________________
    
    float64[6] pos_in            # task pos(posx)  
    int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                                 # <ref is only available in M2.40 or later> 
    int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CoordTransformRequest(null);
    if (msg.pos_in !== undefined) {
      resolved.pos_in = msg.pos_in;
    }
    else {
      resolved.pos_in = new Array(6).fill(0)
    }

    if (msg.ref_in !== undefined) {
      resolved.ref_in = msg.ref_in;
    }
    else {
      resolved.ref_in = 0
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

class CoordTransformResponse {
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
    // Serializes a message object of type CoordTransformResponse
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
    //deserializes a message object of type CoordTransformResponse
    let len;
    let data = new CoordTransformResponse(null);
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
    return 'dsr_msgs/CoordTransformResponse';
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
    const resolved = new CoordTransformResponse(null);
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
  Request: CoordTransformRequest,
  Response: CoordTransformResponse,
  md5sum() { return '0cb6558d600295c3a86e9d114e608d8a'; },
  datatype() { return 'dsr_msgs/CoordTransform'; }
};
