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

class RobotStop {
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
    // Serializes a message object of type RobotStop
    // Serialize message field [stop_mode]
    bufferOffset = _serializer.int32(obj.stop_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStop
    let len;
    let data = new RobotStop(null);
    // Deserialize message field [stop_mode]
    data.stop_mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/RobotStop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82712390efeed0d0668a551e004b332c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # [ robot stop mode ] 
    # 0 : STOP_TYPE_QUICK_STO
    # 1 : STOP_TYPE_QUICK
    # 2 : STOP_TYPE_SLOW
    # 3 : STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY
    #
    #____________________________________________________________________________________________
    
    int32 stop_mode  
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotStop(null);
    if (msg.stop_mode !== undefined) {
      resolved.stop_mode = msg.stop_mode;
    }
    else {
      resolved.stop_mode = 0
    }

    return resolved;
    }
};

module.exports = RobotStop;
