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

class GetCurrentVeljRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentVeljRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentVeljRequest
    let len;
    let data = new GetCurrentVeljRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentVeljRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_current_velj()  
    #____________________________________________________________________________________________
    # This service returns the current target joint velocity. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentVeljRequest(null);
    return resolved;
    }
};

class GetCurrentVeljResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_speed = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_speed')) {
        this.joint_speed = initObj.joint_speed
      }
      else {
        this.joint_speed = new Array(6).fill(0);
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
    // Serializes a message object of type GetCurrentVeljResponse
    // Check that the constant length array field [joint_speed] has the right length
    if (obj.joint_speed.length !== 6) {
      throw new Error('Unable to serialize array field joint_speed - length must be 6')
    }
    // Serialize message field [joint_speed]
    bufferOffset = _arraySerializer.float64(obj.joint_speed, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentVeljResponse
    let len;
    let data = new GetCurrentVeljResponse(null);
    // Deserialize message field [joint_speed]
    data.joint_speed = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentVeljResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9162ee4b17260f95d8a41690c4c86ea1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6]  joint_speed               # joint speed 
    bool        success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentVeljResponse(null);
    if (msg.joint_speed !== undefined) {
      resolved.joint_speed = msg.joint_speed;
    }
    else {
      resolved.joint_speed = new Array(6).fill(0)
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
  Request: GetCurrentVeljRequest,
  Response: GetCurrentVeljResponse,
  md5sum() { return '9162ee4b17260f95d8a41690c4c86ea1'; },
  datatype() { return 'dsr_msgs/GetCurrentVelj'; }
};
