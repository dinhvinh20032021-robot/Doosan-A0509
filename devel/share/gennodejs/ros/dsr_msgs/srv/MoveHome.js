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

class MoveHomeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target = null;
    }
    else {
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveHomeRequest
    // Serialize message field [target]
    bufferOffset = _serializer.int8(obj.target, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveHomeRequest
    let len;
    let data = new MoveHomeRequest(null);
    // Deserialize message field [target]
    data.target = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveHomeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de81b5d46db64f0ac9cb8e5ed77e1d9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_home
    # Homing is performed by moving to the joint motion to the mechanical or user defined home position.
    # According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
    #____________________________________________________________________________________________
    
    int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)
                                # DR_HOME_TARGET_USER(1)     : user home
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveHomeRequest(null);
    if (msg.target !== undefined) {
      resolved.target = msg.target;
    }
    else {
      resolved.target = 0
    }

    return resolved;
    }
};

class MoveHomeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.res = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('res')) {
        this.res = initObj.res
      }
      else {
        this.res = 0;
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
    // Serializes a message object of type MoveHomeResponse
    // Serialize message field [res]
    bufferOffset = _serializer.int8(obj.res, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveHomeResponse
    let len;
    let data = new MoveHomeResponse(null);
    // Deserialize message field [res]
    data.res = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveHomeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '80efef0ff96bf39f322e2d5f77e63c17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8       res              # 0=success, otherwise fail 
    bool       success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveHomeResponse(null);
    if (msg.res !== undefined) {
      resolved.res = msg.res;
    }
    else {
      resolved.res = 0
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
  Request: MoveHomeRequest,
  Response: MoveHomeResponse,
  md5sum() { return '1df045e08f5aa30b46232621f50af2fd'; },
  datatype() { return 'dsr_msgs/MoveHome'; }
};
