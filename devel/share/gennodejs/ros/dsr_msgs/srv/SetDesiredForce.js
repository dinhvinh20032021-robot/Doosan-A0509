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

class SetDesiredForceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fd = null;
      this.dir = null;
      this.ref = null;
      this.time = null;
      this.mod = null;
    }
    else {
      if (initObj.hasOwnProperty('fd')) {
        this.fd = initObj.fd
      }
      else {
        this.fd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('dir')) {
        this.dir = initObj.dir
      }
      else {
        this.dir = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('mod')) {
        this.mod = initObj.mod
      }
      else {
        this.mod = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDesiredForceRequest
    // Check that the constant length array field [fd] has the right length
    if (obj.fd.length !== 6) {
      throw new Error('Unable to serialize array field fd - length must be 6')
    }
    // Serialize message field [fd]
    bufferOffset = _arraySerializer.float64(obj.fd, buffer, bufferOffset, 6);
    // Check that the constant length array field [dir] has the right length
    if (obj.dir.length !== 6) {
      throw new Error('Unable to serialize array field dir - length must be 6')
    }
    // Serialize message field [dir]
    bufferOffset = _arraySerializer.int8(obj.dir, buffer, bufferOffset, 6);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [mod]
    bufferOffset = _serializer.int8(obj.mod, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredForceRequest
    let len;
    let data = new SetDesiredForceRequest(null);
    // Deserialize message field [fd]
    data.fd = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [dir]
    data.dir = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mod]
    data.mod = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetDesiredForceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c08817a2f4a41a3c34c4508a26e9e1a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # set_desired_force  
    #____________________________________________________________________________________________
    
    float64[6] fd                # Three translational target forces + Three rotational target moments
    int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0
    int8       ref               # Reference coordinate of the coordinate to get
    float64    time # 0          # Transition time of target force to take effect (0 ~ 1.0 sec)
    int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, 
                                 # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDesiredForceRequest(null);
    if (msg.fd !== undefined) {
      resolved.fd = msg.fd;
    }
    else {
      resolved.fd = new Array(6).fill(0)
    }

    if (msg.dir !== undefined) {
      resolved.dir = msg.dir;
    }
    else {
      resolved.dir = new Array(6).fill(0)
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.mod !== undefined) {
      resolved.mod = msg.mod;
    }
    else {
      resolved.mod = 0
    }

    return resolved;
    }
};

class SetDesiredForceResponse {
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
    // Serializes a message object of type SetDesiredForceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredForceResponse
    let len;
    let data = new SetDesiredForceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/SetDesiredForceResponse';
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
    const resolved = new SetDesiredForceResponse(null);
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
  Request: SetDesiredForceRequest,
  Response: SetDesiredForceResponse,
  md5sum() { return '06a47c6fc3ae37ec04174ee64d651e04'; },
  datatype() { return 'dsr_msgs/SetDesiredForce'; }
};
