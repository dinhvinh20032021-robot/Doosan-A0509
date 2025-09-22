// Auto-generated. Do not edit!

// (in-package dsr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TorqueRTStream {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tor = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('tor')) {
        this.tor = initObj.tor
      }
      else {
        this.tor = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TorqueRTStream
    // Check that the constant length array field [tor] has the right length
    if (obj.tor.length !== 6) {
      throw new Error('Unable to serialize array field tor - length must be 6')
    }
    // Serialize message field [tor]
    bufferOffset = _arraySerializer.float64(obj.tor, buffer, bufferOffset, 6);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TorqueRTStream
    let len;
    let data = new TorqueRTStream(null);
    // Deserialize message field [tor]
    data.tor = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/TorqueRTStream';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f87591a62093bf8e8f3ad8edb630f87d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # torque_rt
    # 
    #____________________________________________________________________________________________
    
    float64[6] tor               # motor torque
    float64    time              # time
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TorqueRTStream(null);
    if (msg.tor !== undefined) {
      resolved.tor = msg.tor;
    }
    else {
      resolved.tor = new Array(6).fill(0)
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    return resolved;
    }
};

module.exports = TorqueRTStream;
