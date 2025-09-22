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

class ParallelAxis2Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vect = null;
      this.axis = null;
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('vect')) {
        this.vect = initObj.vect
      }
      else {
        this.vect = new Array(3).fill(0);
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
    // Serializes a message object of type ParallelAxis2Request
    // Check that the constant length array field [vect] has the right length
    if (obj.vect.length !== 3) {
      throw new Error('Unable to serialize array field vect - length must be 3')
    }
    // Serialize message field [vect]
    bufferOffset = _arraySerializer.float64(obj.vect, buffer, bufferOffset, 3);
    // Serialize message field [axis]
    bufferOffset = _serializer.int8(obj.axis, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ParallelAxis2Request
    let len;
    let data = new ParallelAxis2Request(null);
    // Deserialize message field [vect]
    data.vect = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [axis]
    data.axis = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ParallelAxis2Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2dace2fbf15dd828aac908612f38504';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # parallel_axis(vect, axis, ref)  
    #____________________________________________________________________________________________
    
    float64[3] vect              # vector[3]  
    int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
    int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ParallelAxis2Request(null);
    if (msg.vect !== undefined) {
      resolved.vect = msg.vect;
    }
    else {
      resolved.vect = new Array(3).fill(0)
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

class ParallelAxis2Response {
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
    // Serializes a message object of type ParallelAxis2Response
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ParallelAxis2Response
    let len;
    let data = new ParallelAxis2Response(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ParallelAxis2Response';
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
    const resolved = new ParallelAxis2Response(null);
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
  Request: ParallelAxis2Request,
  Response: ParallelAxis2Response,
  md5sum() { return '270e0e3f7b07b24362789c76324d4ac4'; },
  datatype() { return 'dsr_msgs/ParallelAxis2'; }
};
