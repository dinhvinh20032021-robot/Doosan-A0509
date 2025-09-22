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

class IsDoneBoltTighteningRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.m = null;
      this.timeout = null;
      this.axis = null;
    }
    else {
      if (initObj.hasOwnProperty('m')) {
        this.m = initObj.m
      }
      else {
        this.m = 0.0;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0.0;
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsDoneBoltTighteningRequest
    // Serialize message field [m]
    bufferOffset = _serializer.float64(obj.m, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.float64(obj.timeout, buffer, bufferOffset);
    // Serialize message field [axis]
    bufferOffset = _serializer.int8(obj.axis, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsDoneBoltTighteningRequest
    let len;
    let data = new IsDoneBoltTighteningRequest(null);
    // Deserialize message field [m]
    data.m = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [axis]
    data.axis = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IsDoneBoltTighteningRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0764049b1e7416d143f065a65482f344';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # is_done_bolt_tightening  
    #____________________________________________________________________________________________
    
    float64    m                 # Target torque  
    float64    timeout           # Monitoring duration [sec]  
    int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsDoneBoltTighteningRequest(null);
    if (msg.m !== undefined) {
      resolved.m = msg.m;
    }
    else {
      resolved.m = 0.0
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0.0
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = 0
    }

    return resolved;
    }
};

class IsDoneBoltTighteningResponse {
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
    // Serializes a message object of type IsDoneBoltTighteningResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsDoneBoltTighteningResponse
    let len;
    let data = new IsDoneBoltTighteningResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/IsDoneBoltTighteningResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsDoneBoltTighteningResponse(null);
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
  Request: IsDoneBoltTighteningRequest,
  Response: IsDoneBoltTighteningResponse,
  md5sum() { return 'be84affb8d741115ea457a393a21b8b5'; },
  datatype() { return 'dsr_msgs/IsDoneBoltTightening'; }
};
