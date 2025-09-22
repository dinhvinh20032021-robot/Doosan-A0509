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

class AlignAxis2Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_vect = null;
      this.source_vect = null;
      this.axis = null;
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('target_vect')) {
        this.target_vect = initObj.target_vect
      }
      else {
        this.target_vect = new Array(3).fill(0);
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
    // Serializes a message object of type AlignAxis2Request
    // Check that the constant length array field [target_vect] has the right length
    if (obj.target_vect.length !== 3) {
      throw new Error('Unable to serialize array field target_vect - length must be 3')
    }
    // Serialize message field [target_vect]
    bufferOffset = _arraySerializer.float64(obj.target_vect, buffer, bufferOffset, 3);
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
    //deserializes a message object of type AlignAxis2Request
    let len;
    let data = new AlignAxis2Request(null);
    // Deserialize message field [target_vect]
    data.target_vect = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [source_vect]
    data.source_vect = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [axis]
    data.axis = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 50;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/AlignAxis2Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03be7d4f574c1507e019c70fb99362be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # align_axis(target_vect, source_vect, axis, ref)
    #____________________________________________________________________________________________
    
    float64[3] target_vect       # target vector[3]  
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
    const resolved = new AlignAxis2Request(null);
    if (msg.target_vect !== undefined) {
      resolved.target_vect = msg.target_vect;
    }
    else {
      resolved.target_vect = new Array(3).fill(0)
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

class AlignAxis2Response {
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
    // Serializes a message object of type AlignAxis2Response
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AlignAxis2Response
    let len;
    let data = new AlignAxis2Response(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/AlignAxis2Response';
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
    const resolved = new AlignAxis2Response(null);
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
  Request: AlignAxis2Request,
  Response: AlignAxis2Response,
  md5sum() { return '789e8de44f005548a96c6375678d766c'; },
  datatype() { return 'dsr_msgs/AlignAxis2'; }
};
