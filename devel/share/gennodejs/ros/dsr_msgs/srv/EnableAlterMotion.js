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

class EnableAlterMotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n = null;
      this.mode = null;
      this.ref = null;
      this.limit_dPOS = null;
      this.limit_dPOS_per = null;
    }
    else {
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('limit_dPOS')) {
        this.limit_dPOS = initObj.limit_dPOS
      }
      else {
        this.limit_dPOS = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('limit_dPOS_per')) {
        this.limit_dPOS_per = initObj.limit_dPOS_per
      }
      else {
        this.limit_dPOS_per = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableAlterMotionRequest
    // Serialize message field [n]
    bufferOffset = _serializer.int32(obj.n, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Check that the constant length array field [limit_dPOS] has the right length
    if (obj.limit_dPOS.length !== 2) {
      throw new Error('Unable to serialize array field limit_dPOS - length must be 2')
    }
    // Serialize message field [limit_dPOS]
    bufferOffset = _arraySerializer.float64(obj.limit_dPOS, buffer, bufferOffset, 2);
    // Check that the constant length array field [limit_dPOS_per] has the right length
    if (obj.limit_dPOS_per.length !== 2) {
      throw new Error('Unable to serialize array field limit_dPOS_per - length must be 2')
    }
    // Serialize message field [limit_dPOS_per]
    bufferOffset = _arraySerializer.float64(obj.limit_dPOS_per, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableAlterMotionRequest
    let len;
    let data = new EnableAlterMotionRequest(null);
    // Deserialize message field [n]
    data.n = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [limit_dPOS]
    data.limit_dPOS = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [limit_dPOS_per]
    data.limit_dPOS_per = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/EnableAlterMotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29de75ebd8f3d681fcf3acce7db9a9d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # enable_alter_motion  
    # 
    #____________________________________________________________________________________________
    
    int32      n                 # Cycle time number 
    int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount 
    int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                                 # <ref is only available in M2.40 or later> 
    float64[2] limit_dPOS        # First value : limitation of position[mm], Second value : limitation of orientation[deg]
    float64[2] limit_dPOS_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableAlterMotionRequest(null);
    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.limit_dPOS !== undefined) {
      resolved.limit_dPOS = msg.limit_dPOS;
    }
    else {
      resolved.limit_dPOS = new Array(2).fill(0)
    }

    if (msg.limit_dPOS_per !== undefined) {
      resolved.limit_dPOS_per = msg.limit_dPOS_per;
    }
    else {
      resolved.limit_dPOS_per = new Array(2).fill(0)
    }

    return resolved;
    }
};

class EnableAlterMotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableAlterMotionResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableAlterMotionResponse
    let len;
    let data = new EnableAlterMotionResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/EnableAlterMotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableAlterMotionResponse(null);
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
  Request: EnableAlterMotionRequest,
  Response: EnableAlterMotionResponse,
  md5sum() { return '4bc090a8b26dd2353ca78fe889287b7f'; },
  datatype() { return 'dsr_msgs/EnableAlterMotion'; }
};
