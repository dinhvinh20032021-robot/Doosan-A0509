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

class GetJointTorqueRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJointTorqueRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointTorqueRequest
    let len;
    let data = new GetJointTorqueRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetJointTorqueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_joint_torque()
    # returns the sensor torque value of the current joint.
    #____________________________________________________________________________________________
    # This service returns the sensor torque value of the current joint.
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointTorqueRequest(null);
    return resolved;
    }
};

class GetJointTorqueResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jts = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('jts')) {
        this.jts = initObj.jts
      }
      else {
        this.jts = new Array(6).fill(0);
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
    // Serializes a message object of type GetJointTorqueResponse
    // Check that the constant length array field [jts] has the right length
    if (obj.jts.length !== 6) {
      throw new Error('Unable to serialize array field jts - length must be 6')
    }
    // Serialize message field [jts]
    bufferOffset = _arraySerializer.float64(obj.jts, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointTorqueResponse
    let len;
    let data = new GetJointTorqueResponse(null);
    // Deserialize message field [jts]
    data.jts = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetJointTorqueResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2bba428be693e219072ca1b9c1a000a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] jts         # value of JTS(Joint Torque Sensor) 
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointTorqueResponse(null);
    if (msg.jts !== undefined) {
      resolved.jts = msg.jts;
    }
    else {
      resolved.jts = new Array(6).fill(0)
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
  Request: GetJointTorqueRequest,
  Response: GetJointTorqueResponse,
  md5sum() { return 'd2bba428be693e219072ca1b9c1a000a'; },
  datatype() { return 'dsr_msgs/GetJointTorque'; }
};
