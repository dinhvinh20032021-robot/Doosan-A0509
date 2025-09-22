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

class JogMultiAxis {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jog_axis = null;
      this.move_reference = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('jog_axis')) {
        this.jog_axis = initObj.jog_axis
      }
      else {
        this.jog_axis = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('move_reference')) {
        this.move_reference = initObj.move_reference
      }
      else {
        this.move_reference = 0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JogMultiAxis
    // Check that the constant length array field [jog_axis] has the right length
    if (obj.jog_axis.length !== 6) {
      throw new Error('Unable to serialize array field jog_axis - length must be 6')
    }
    // Serialize message field [jog_axis]
    bufferOffset = _arraySerializer.float64(obj.jog_axis, buffer, bufferOffset, 6);
    // Serialize message field [move_reference]
    bufferOffset = _serializer.int8(obj.move_reference, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JogMultiAxis
    let len;
    let data = new JogMultiAxis(null);
    // Deserialize message field [jog_axis]
    data.jog_axis = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [move_reference]
    data.move_reference = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dsr_msgs/JogMultiAxis';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e36e615ff2ffad91791b721818ab6be9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # multi jog
    # multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] 
    #____________________________________________________________________________________________
    
    float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
    int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
    float64     speed             # jog speed [%]
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JogMultiAxis(null);
    if (msg.jog_axis !== undefined) {
      resolved.jog_axis = msg.jog_axis;
    }
    else {
      resolved.jog_axis = new Array(6).fill(0)
    }

    if (msg.move_reference !== undefined) {
      resolved.move_reference = msg.move_reference;
    }
    else {
      resolved.move_reference = 0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    return resolved;
    }
};

module.exports = JogMultiAxis;
