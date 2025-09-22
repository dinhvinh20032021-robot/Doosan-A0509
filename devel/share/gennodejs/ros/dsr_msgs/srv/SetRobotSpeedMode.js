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

class SetRobotSpeedModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('speed_mode')) {
        this.speed_mode = initObj.speed_mode
      }
      else {
        this.speed_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRobotSpeedModeRequest
    // Serialize message field [speed_mode]
    bufferOffset = _serializer.int8(obj.speed_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotSpeedModeRequest
    let len;
    let data = new SetRobotSpeedModeRequest(null);
    // Deserialize message field [speed_mode]
    data.speed_mode = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotSpeedModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '19b88c6c53fbc4047e5743c63539f425';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_robot_speed_mode
    #____________________________________________________________________________________________
    
    int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRobotSpeedModeRequest(null);
    if (msg.speed_mode !== undefined) {
      resolved.speed_mode = msg.speed_mode;
    }
    else {
      resolved.speed_mode = 0
    }

    return resolved;
    }
};

class SetRobotSpeedModeResponse {
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
    // Serializes a message object of type SetRobotSpeedModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRobotSpeedModeResponse
    let len;
    let data = new SetRobotSpeedModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetRobotSpeedModeResponse';
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
    const resolved = new SetRobotSpeedModeResponse(null);
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
  Request: SetRobotSpeedModeRequest,
  Response: SetRobotSpeedModeResponse,
  md5sum() { return '724227601ad811bee3732775fc3c79d1'; },
  datatype() { return 'dsr_msgs/SetRobotSpeedMode'; }
};
