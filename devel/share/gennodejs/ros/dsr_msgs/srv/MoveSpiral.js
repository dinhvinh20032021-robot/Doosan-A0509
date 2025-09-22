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

class MoveSpiralRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.revolution = null;
      this.maxRadius = null;
      this.maxLength = null;
      this.vel = null;
      this.acc = null;
      this.time = null;
      this.taskAxis = null;
      this.ref = null;
      this.syncType = null;
    }
    else {
      if (initObj.hasOwnProperty('revolution')) {
        this.revolution = initObj.revolution
      }
      else {
        this.revolution = 0.0;
      }
      if (initObj.hasOwnProperty('maxRadius')) {
        this.maxRadius = initObj.maxRadius
      }
      else {
        this.maxRadius = 0.0;
      }
      if (initObj.hasOwnProperty('maxLength')) {
        this.maxLength = initObj.maxLength
      }
      else {
        this.maxLength = 0.0;
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('taskAxis')) {
        this.taskAxis = initObj.taskAxis
      }
      else {
        this.taskAxis = 0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
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
    // Serializes a message object of type MoveSpiralRequest
    // Serialize message field [revolution]
    bufferOffset = _serializer.float64(obj.revolution, buffer, bufferOffset);
    // Serialize message field [maxRadius]
    bufferOffset = _serializer.float64(obj.maxRadius, buffer, bufferOffset);
    // Serialize message field [maxLength]
    bufferOffset = _serializer.float64(obj.maxLength, buffer, bufferOffset);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 2) {
      throw new Error('Unable to serialize array field vel - length must be 2')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, 2);
    // Check that the constant length array field [acc] has the right length
    if (obj.acc.length !== 2) {
      throw new Error('Unable to serialize array field acc - length must be 2')
    }
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, 2);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [taskAxis]
    bufferOffset = _serializer.int8(obj.taskAxis, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [syncType]
    bufferOffset = _serializer.int8(obj.syncType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveSpiralRequest
    let len;
    let data = new MoveSpiralRequest(null);
    // Deserialize message field [revolution]
    data.revolution = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [maxRadius]
    data.maxRadius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [maxLength]
    data.maxLength = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [taskAxis]
    data.taskAxis = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [syncType]
    data.syncType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 67;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveSpiralRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5d20768cc7c6e3fb127c70920255d0e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_sprial  
    #____________________________________________________________________________________________
    
    float64    revolution       # Total number of revolutions 
    float64    maxRadius        # Final spiral radius [mm]
    float64    maxLength        # Distance moved in the axis direction [mm]
    float64[2] vel              # set velocity: [mm/sec], [deg/sec]
    float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]
    float64    time #= 0.0      # Total execution time <sec> 
    int8       taskAxis         # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   
    int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                # <DR_WORLD is only available in M2.40 or later 
    int8       syncType #=0     # SYNC = 0, ASYNC = 1 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveSpiralRequest(null);
    if (msg.revolution !== undefined) {
      resolved.revolution = msg.revolution;
    }
    else {
      resolved.revolution = 0.0
    }

    if (msg.maxRadius !== undefined) {
      resolved.maxRadius = msg.maxRadius;
    }
    else {
      resolved.maxRadius = 0.0
    }

    if (msg.maxLength !== undefined) {
      resolved.maxLength = msg.maxLength;
    }
    else {
      resolved.maxLength = 0.0
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = new Array(2).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = new Array(2).fill(0)
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.taskAxis !== undefined) {
      resolved.taskAxis = msg.taskAxis;
    }
    else {
      resolved.taskAxis = 0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
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

class MoveSpiralResponse {
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
    // Serializes a message object of type MoveSpiralResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveSpiralResponse
    let len;
    let data = new MoveSpiralResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveSpiralResponse';
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
    const resolved = new MoveSpiralResponse(null);
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
  Request: MoveSpiralRequest,
  Response: MoveSpiralResponse,
  md5sum() { return 'fc2c602cd43258b5eb06926d790138ea'; },
  datatype() { return 'dsr_msgs/MoveSpiral'; }
};
