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

class GetRobotSystemRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotSystemRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotSystemRequest
    let len;
    let data = new GetRobotSystemRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetRobotSystemRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_robot_system
    #____________________________________________________________________________________________
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotSystemRequest(null);
    return resolved;
    }
};

class GetRobotSystemResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_system = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_system')) {
        this.robot_system = initObj.robot_system
      }
      else {
        this.robot_system = 0;
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
    // Serializes a message object of type GetRobotSystemResponse
    // Serialize message field [robot_system]
    bufferOffset = _serializer.int8(obj.robot_system, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotSystemResponse
    let len;
    let data = new GetRobotSystemResponse(null);
    // Deserialize message field [robot_system]
    data.robot_system = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetRobotSystemResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20308dcb70c4e3f1df099ac95a676cf2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                        # 1 : ROBOT_SYSTEM_VIRTUAL
    bool        success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotSystemResponse(null);
    if (msg.robot_system !== undefined) {
      resolved.robot_system = msg.robot_system;
    }
    else {
      resolved.robot_system = 0
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
  Request: GetRobotSystemRequest,
  Response: GetRobotSystemResponse,
  md5sum() { return '20308dcb70c4e3f1df099ac95a676cf2'; },
  datatype() { return 'dsr_msgs/GetRobotSystem'; }
};
