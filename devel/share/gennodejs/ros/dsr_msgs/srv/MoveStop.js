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

class MoveStopRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stop_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('stop_mode')) {
        this.stop_mode = initObj.stop_mode
      }
      else {
        this.stop_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveStopRequest
    // Serialize message field [stop_mode]
    bufferOffset = _serializer.int32(obj.stop_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveStopRequest
    let len;
    let data = new MoveStopRequest(null);
    // Deserialize message field [stop_mode]
    data.stop_mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveStopRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82712390efeed0d0668a551e004b332c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # stop()
    # 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #____________________________________________________________________________________________
    
    int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)
                            # DR_QSTOP(1)     : Quick stop (Stop Category 2)
                            # DR_SSTO(2)      : Soft Stop
                            # DR_HOLD(3)      : HOLD stop
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveStopRequest(null);
    if (msg.stop_mode !== undefined) {
      resolved.stop_mode = msg.stop_mode;
    }
    else {
      resolved.stop_mode = 0
    }

    return resolved;
    }
};

class MoveStopResponse {
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
    // Serializes a message object of type MoveStopResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveStopResponse
    let len;
    let data = new MoveStopResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveStopResponse';
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
    const resolved = new MoveStopResponse(null);
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
  Request: MoveStopRequest,
  Response: MoveStopResponse,
  md5sum() { return 'a0a25e9e76d4431108e6e2682aa29331'; },
  datatype() { return 'dsr_msgs/MoveStop'; }
};
