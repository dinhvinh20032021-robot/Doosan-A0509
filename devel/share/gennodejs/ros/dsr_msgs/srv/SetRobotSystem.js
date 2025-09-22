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

class SetRobotSystemRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_system = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_system')) {
        this.robot_system = initObj.robot_system
      }
      else {
        this.robot_system = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotSystemRequest
    // Serialize message field [robot_system]
    bufferOffset = _serializer.int8(obj.robot_system, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotSystemRequest
    let len;
    let data = new SetRobotSystemRequest(null);
    // Deserialize message field [robot_system]
    data.robot_system = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotSystemRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95994932864febcd35ae02067672f216';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_robot_system
    #____________________________________________________________________________________________
    
    int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotSystemRequest(null);
    if (msg.robot_system !== undefined) {
      resolved.robot_system = msg.robot_system;
    }
    else {
      resolved.robot_system = 0
    }

    return resolved;
    }
};

class SetRobotSystemResponse {
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
    // Serializes a message object of type SetRobotSystemResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotSystemResponse
    let len;
    let data = new SetRobotSystemResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotSystemResponse';
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
    const resolved = new SetRobotSystemResponse(null);
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
  Request: SetRobotSystemRequest,
  Response: SetRobotSystemResponse,
  md5sum() { return '5474fc13fdf01710b0f371e4b3b11c86'; },
  datatype() { return 'dsr_msgs/SetRobotSystem'; }
};
