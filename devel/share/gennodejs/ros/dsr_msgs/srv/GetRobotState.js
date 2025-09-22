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

class GetRobotStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotStateRequest
    let len;
    let data = new GetRobotStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetRobotStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_robot_state
    #____________________________________________________________________________________________
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotStateRequest(null);
    return resolved;
    }
};

class GetRobotStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_state = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = 0;
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
    // Serializes a message object of type GetRobotStateResponse
    // Serialize message field [robot_state]
    bufferOffset = _serializer.int8(obj.robot_state, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotStateResponse
    let len;
    let data = new GetRobotStateResponse(null);
    // Deserialize message field [robot_state]
    data.robot_state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetRobotStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55f09762597f0e81f6f46902de1fad06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 robot_state    # 0 : STATE_INITIALIZING
                        # 1 : STATE_STANDBY
                        # 2 : STATE_MOVING
                        # 3 : STATE_SAFE_OFF
                        # 4 : STATE_TEACHING
                        # 5 : STATE_SAFE_STOP
                        # 6 : STATE_EMERGENCY_STOP:
                        # 7 : STATE_HOMMING
                        # 8 : STATE_RECOVERY
                        # 9 : eSTATE_SAFE_STOP2
                        # 10: STATE_SAFE_OFF2
                        # 11: STATE_RESERVED1
                        # 12: STATE_RESERVED2
                        # 13: STATE_RESERVED3
                        # 14: STATE_RESERVED4
                        # 15: STATE_NOT_READY
    bool        success                    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotStateResponse(null);
    if (msg.robot_state !== undefined) {
      resolved.robot_state = msg.robot_state;
    }
    else {
      resolved.robot_state = 0
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
  Request: GetRobotStateRequest,
  Response: GetRobotStateResponse,
  md5sum() { return '55f09762597f0e81f6f46902de1fad06'; },
  datatype() { return 'dsr_msgs/GetRobotState'; }
};
