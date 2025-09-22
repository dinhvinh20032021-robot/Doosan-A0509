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

class GetDesiredVeljRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDesiredVeljRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDesiredVeljRequest
    let len;
    let data = new GetDesiredVeljRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetDesiredVeljRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_desired_velj()  
    #____________________________________________________________________________________________
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDesiredVeljRequest(null);
    return resolved;
    }
};

class GetDesiredVeljResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_vel = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_vel')) {
        this.joint_vel = initObj.joint_vel
      }
      else {
        this.joint_vel = new Array(6).fill(0);
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
    // Serializes a message object of type GetDesiredVeljResponse
    // Check that the constant length array field [joint_vel] has the right length
    if (obj.joint_vel.length !== 6) {
      throw new Error('Unable to serialize array field joint_vel - length must be 6')
    }
    // Serialize message field [joint_vel]
    bufferOffset = _arraySerializer.float64(obj.joint_vel, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDesiredVeljResponse
    let len;
    let data = new GetDesiredVeljResponse(null);
    // Deserialize message field [joint_vel]
    data.joint_vel = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetDesiredVeljResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88bf0c6c29b23cd493e33b3354d4f21a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] joint_vel               # Target joint velocity 
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDesiredVeljResponse(null);
    if (msg.joint_vel !== undefined) {
      resolved.joint_vel = msg.joint_vel;
    }
    else {
      resolved.joint_vel = new Array(6).fill(0)
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
  Request: GetDesiredVeljRequest,
  Response: GetDesiredVeljResponse,
  md5sum() { return '88bf0c6c29b23cd493e33b3354d4f21a'; },
  datatype() { return 'dsr_msgs/GetDesiredVelj'; }
};
