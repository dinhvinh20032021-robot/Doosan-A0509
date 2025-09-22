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

class SetRobotControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_control = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_control')) {
        this.robot_control = initObj.robot_control
      }
      else {
        this.robot_control = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotControlRequest
    // Serialize message field [robot_control]
    bufferOffset = _serializer.int8(obj.robot_control, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotControlRequest
    let len;
    let data = new SetRobotControlRequest(null);
    // Deserialize message field [robot_control]
    data.robot_control = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69dab02916cb4890b887d4a33beaa620';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_robot_control
    #____________________________________________________________________________________________
    
    int8 robot_control # 0 : CONTROL_INIT_CONFIG
                       # 1 : CONTROL_ENABLE_OPERATION
                       # 2 : CONTROL_RESET_SAFET_STOP
                       # 3 : CONTROL_RESET_SAFE_STOP = CONTROL_RESET_SAFET_STOP,
                       # 4 : CONTROL_RESET_SAFET_OFF,
                       # 5 : CONTROL_RESET_SAFE_OFF = CONTROL_RESET_SAFET_OFF,
                       # 6 : CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF,
                       # 7 : CONTROL_RECOVERY_SAFE_STOP,
                       # 8 : CONTROL_RECOVERY_SAFE_OFF,
                       # 9 : CONTROL_RECOVERY_BACKDRIVE,
                       # 10: CONTROL_RESET_RECOVERY,
                       # 11: CONTROL_LAST
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotControlRequest(null);
    if (msg.robot_control !== undefined) {
      resolved.robot_control = msg.robot_control;
    }
    else {
      resolved.robot_control = 0
    }

    return resolved;
    }
};

class SetRobotControlResponse {
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
    // Serializes a message object of type SetRobotControlResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotControlResponse
    let len;
    let data = new SetRobotControlResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotControlResponse';
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
    const resolved = new SetRobotControlResponse(null);
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
  Request: SetRobotControlRequest,
  Response: SetRobotControlResponse,
  md5sum() { return 'e3c7c3cfa0120a8cf83abd30ec7bcbc3'; },
  datatype() { return 'dsr_msgs/SetRobotControl'; }
};
