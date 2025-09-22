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

class MoveJointxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.vel = null;
      this.acc = null;
      this.time = null;
      this.radius = null;
      this.ref = null;
      this.mode = null;
      this.blendType = null;
      this.sol = null;
      this.syncType = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = 0.0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('blendType')) {
        this.blendType = initObj.blendType
      }
      else {
        this.blendType = 0;
      }
      if (initObj.hasOwnProperty('sol')) {
        this.sol = initObj.sol
      }
      else {
        this.sol = 0;
      }
      if (initObj.hasOwnProperty('syncType')) {
        this.syncType = initObj.syncType
      }
      else {
        this.syncType = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveJointxRequest
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 6) {
      throw new Error('Unable to serialize array field pos - length must be 6')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 6);
    // Serialize message field [vel]
    bufferOffset = _serializer.float64(obj.vel, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = _serializer.float64(obj.acc, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Serialize message field [blendType]
    bufferOffset = _serializer.int8(obj.blendType, buffer, bufferOffset);
    // Serialize message field [sol]
    bufferOffset = _serializer.int8(obj.sol, buffer, bufferOffset);
    // Serialize message field [syncType]
    bufferOffset = _serializer.int8(obj.syncType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointxRequest
    let len;
    let data = new MoveJointxRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [vel]
    data.vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [blendType]
    data.blendType = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [sol]
    data.sol = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [syncType]
    data.syncType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 85;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveJointxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '990b77a624453e4a3faf362b98093629';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_jointx  
    #____________________________________________________________________________________________
    
    float64[6] pos              # target  
    float64    vel              # set velocity: [deg/sec]
    float64    acc              # set acceleration: [deg/sec2] 
    float64    time #= 0.0      # Time [sec] 
    float64    radius #=0.0     # Radius under blending mode [mm]   
    int8       ref              # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                # <DR_WORLD is only available in M2.40 or later> 
    int8       mode #= 0        # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
    int8       blendType #= 0   # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
    int8       sol              # SolutionSpace : 0~7
    int8       syncType #=0     # SYNC = 0, ASYNC = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveJointxRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(6).fill(0)
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = 0.0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.blendType !== undefined) {
      resolved.blendType = msg.blendType;
    }
    else {
      resolved.blendType = 0
    }

    if (msg.sol !== undefined) {
      resolved.sol = msg.sol;
    }
    else {
      resolved.sol = 0
    }

    if (msg.syncType !== undefined) {
      resolved.syncType = msg.syncType;
    }
    else {
      resolved.syncType = 0
    }

    return resolved;
    }
};

class MoveJointxResponse {
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
    // Serializes a message object of type MoveJointxResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointxResponse
    let len;
    let data = new MoveJointxResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveJointxResponse';
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
    const resolved = new MoveJointxResponse(null);
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
  Request: MoveJointxRequest,
  Response: MoveJointxResponse,
  md5sum() { return 'b9419696b37ce5162db29de34d49184f'; },
  datatype() { return 'dsr_msgs/MoveJointx'; }
};
