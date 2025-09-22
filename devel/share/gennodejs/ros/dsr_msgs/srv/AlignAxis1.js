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

class AlignAxis1Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x1 = null;
      this.x2 = null;
      this.x3 = null;
      this.source_vect = null;
      this.axis = null;
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('x1')) {
        this.x1 = initObj.x1
      }
      else {
        this.x1 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('x2')) {
        this.x2 = initObj.x2
      }
      else {
        this.x2 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('x3')) {
        this.x3 = initObj.x3
      }
      else {
        this.x3 = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('source_vect')) {
        this.source_vect = initObj.source_vect
      }
      else {
        this.source_vect = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = 0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AlignAxis1Request
    // Check that the constant length array field [x1] has the right length
    if (obj.x1.length !== 6) {
      throw new Error('Unable to serialize array field x1 - length must be 6')
    }
    // Serialize message field [x1]
    bufferOffset = _arraySerializer.float64(obj.x1, buffer, bufferOffset, 6);
    // Check that the constant length array field [x2] has the right length
    if (obj.x2.length !== 6) {
      throw new Error('Unable to serialize array field x2 - length must be 6')
    }
    // Serialize message field [x2]
    bufferOffset = _arraySerializer.float64(obj.x2, buffer, bufferOffset, 6);
    // Check that the constant length array field [x3] has the right length
    if (obj.x3.length !== 6) {
      throw new Error('Unable to serialize array field x3 - length must be 6')
    }
    // Serialize message field [x3]
    bufferOffset = _arraySerializer.float64(obj.x3, buffer, bufferOffset, 6);
    // Check that the constant length array field [source_vect] has the right length
    if (obj.source_vect.length !== 3) {
      throw new Error('Unable to serialize array field source_vect - length must be 3')
    }
    // Serialize message field [source_vect]
    bufferOffset = _arraySerializer.float64(obj.source_vect, buffer, bufferOffset, 3);
    // Serialize message field [axis]
    bufferOffset = _serializer.int8(obj.axis, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AlignAxis1Request
    let len;
    let data = new AlignAxis1Request(null);
    // Deserialize message field [x1]
    data.x1 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x2]
    data.x2 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [x3]
    data.x3 = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [source_vect]
    data.source_vect = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [axis]
    data.axis = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 170;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/AlignAxis1Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b58a6b9aa751aa4ac3b4fa16f7db70aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # align_axis(x1, x2, x3, source_vect, axis, ref)
    #____________________________________________________________________________________________
    
    float64[6] x1                # task pos(posx)  
    float64[6] x2                # task pos(posx)  
    float64[6] x3                # task pos(posx)
    float64[3] source_vect       # source vector[3]  
    int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
    int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AlignAxis1Request(null);
    if (msg.x1 !== undefined) {
      resolved.x1 = msg.x1;
    }
    else {
      resolved.x1 = new Array(6).fill(0)
    }

    if (msg.x2 !== undefined) {
      resolved.x2 = msg.x2;
    }
    else {
      resolved.x2 = new Array(6).fill(0)
    }

    if (msg.x3 !== undefined) {
      resolved.x3 = msg.x3;
    }
    else {
      resolved.x3 = new Array(6).fill(0)
    }

    if (msg.source_vect !== undefined) {
      resolved.source_vect = msg.source_vect;
    }
    else {
      resolved.source_vect = new Array(3).fill(0)
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = 0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    return resolved;
    }
};

class AlignAxis1Response {
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
    // Serializes a message object of type AlignAxis1Response
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AlignAxis1Response
    let len;
    let data = new AlignAxis1Response(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/AlignAxis1Response';
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
    const resolved = new AlignAxis1Response(null);
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
  Request: AlignAxis1Request,
  Response: AlignAxis1Response,
  md5sum() { return 'bc982fa1299790397f690022cfb7030c'; },
  datatype() { return 'dsr_msgs/AlignAxis1'; }
};
