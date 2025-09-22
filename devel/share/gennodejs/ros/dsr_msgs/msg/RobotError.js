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

class RobotError {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.level = null;
      this.group = null;
      this.code = null;
      this.msg1 = null;
      this.msg2 = null;
      this.msg3 = null;
    }
    else {
      if (initObj.hasOwnProperty('level')) {
        this.level = initObj.level
      }
      else {
        this.level = 0;
      }
      if (initObj.hasOwnProperty('group')) {
        this.group = initObj.group
      }
      else {
        this.group = 0;
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('msg1')) {
        this.msg1 = initObj.msg1
      }
      else {
        this.msg1 = '';
      }
      if (initObj.hasOwnProperty('msg2')) {
        this.msg2 = initObj.msg2
      }
      else {
        this.msg2 = '';
      }
      if (initObj.hasOwnProperty('msg3')) {
        this.msg3 = initObj.msg3
      }
      else {
        this.msg3 = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotError
    // Serialize message field [level]
    bufferOffset = _serializer.int32(obj.level, buffer, bufferOffset);
    // Serialize message field [group]
    bufferOffset = _serializer.int32(obj.group, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.int32(obj.code, buffer, bufferOffset);
    // Serialize message field [msg1]
    bufferOffset = _serializer.string(obj.msg1, buffer, bufferOffset);
    // Serialize message field [msg2]
    bufferOffset = _serializer.string(obj.msg2, buffer, bufferOffset);
    // Serialize message field [msg3]
    bufferOffset = _serializer.string(obj.msg3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotError
    let len;
    let data = new RobotError(null);
    // Deserialize message field [level]
    data.level = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [group]
    data.group = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [msg1]
    data.msg1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [msg2]
    data.msg2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [msg3]
    data.msg3 = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.msg1);
    length += _getByteLength(object.msg2);
    length += _getByteLength(object.msg3);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/RobotError';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d9680ace6867f719d7e19bbc321e6da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # [ robot error msg ] 
    #____________________________________________________________________________________________
    
    int32	    level   # INFO =1, WARN =2, ERROR =3 
    int32	    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int32	    code    # error code 
    string      msg1    # error msg 1
    string      msg2    # error msg 2
    string      msg3    # error msg 3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotError(null);
    if (msg.level !== undefined) {
      resolved.level = msg.level;
    }
    else {
      resolved.level = 0
    }

    if (msg.group !== undefined) {
      resolved.group = msg.group;
    }
    else {
      resolved.group = 0
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.msg1 !== undefined) {
      resolved.msg1 = msg.msg1;
    }
    else {
      resolved.msg1 = ''
    }

    if (msg.msg2 !== undefined) {
      resolved.msg2 = msg.msg2;
    }
    else {
      resolved.msg2 = ''
    }

    if (msg.msg3 !== undefined) {
      resolved.msg3 = msg.msg3;
    }
    else {
      resolved.msg3 = ''
    }

    return resolved;
    }
};

module.exports = RobotError;
