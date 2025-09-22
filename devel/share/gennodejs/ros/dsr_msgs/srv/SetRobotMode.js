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

class SetRobotModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_mode')) {
        this.robot_mode = initObj.robot_mode
      }
      else {
        this.robot_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotModeRequest
    // Serialize message field [robot_mode]
    bufferOffset = _serializer.int8(obj.robot_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotModeRequest
    let len;
    let data = new SetRobotModeRequest(null);
    // Deserialize message field [robot_mode]
    data.robot_mode = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e693f8d2a4951cec41fb83aa5891c822';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_robot_mode
    # Change the robot-mode
    # 0 : ROBOT_MODE_MANUAL
    # 1 : ROBOT_MODE_AUTONOMOUS
    # 2 :ROBOT_MODE_MEASURE
    # drfl.SetRobotMode()
    #____________________________________________________________________________________________
    
    int8 robot_mode # <Robot_Mode>
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotModeRequest(null);
    if (msg.robot_mode !== undefined) {
      resolved.robot_mode = msg.robot_mode;
    }
    else {
      resolved.robot_mode = 0
    }

    return resolved;
    }
};

class SetRobotModeResponse {
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
    // Serializes a message object of type SetRobotModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotModeResponse
    let len;
    let data = new SetRobotModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotModeResponse';
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
    const resolved = new SetRobotModeResponse(null);
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
  Request: SetRobotModeRequest,
  Response: SetRobotModeResponse,
  md5sum() { return '1839172555f03e6135521c0cb3b452f9'; },
  datatype() { return 'dsr_msgs/SetRobotMode'; }
};
