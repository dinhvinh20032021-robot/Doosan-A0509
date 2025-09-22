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

class GetOrientationErrorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.xd = null;
      this.xc = null;
      this.axis = null;
    }
    else {
      if (initObj.hasOwnProperty('xd')) {
        this.xd = initObj.xd
      }
      else {
        this.xd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('xc')) {
        this.xc = initObj.xc
      }
      else {
        this.xc = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOrientationErrorRequest
    // Check that the constant length array field [xd] has the right length
    if (obj.xd.length !== 6) {
      throw new Error('Unable to serialize array field xd - length must be 6')
    }
    // Serialize message field [xd]
    bufferOffset = _arraySerializer.float64(obj.xd, buffer, bufferOffset, 6);
    // Check that the constant length array field [xc] has the right length
    if (obj.xc.length !== 6) {
      throw new Error('Unable to serialize array field xc - length must be 6')
    }
    // Serialize message field [xc]
    bufferOffset = _arraySerializer.float64(obj.xc, buffer, bufferOffset, 6);
    // Serialize message field [axis]
    bufferOffset = _serializer.int8(obj.axis, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOrientationErrorRequest
    let len;
    let data = new GetOrientationErrorRequest(null);
    // Deserialize message field [xd]
    data.xd = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [xc]
    data.xc = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [axis]
    data.axis = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 97;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetOrientationErrorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f5697202e6cec6fea98c82889255f7e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_orientation_error  
    #____________________________________________________________________________________________
    # This service returns the orientation error value between the arbitrary poses xd and xc of the axis.
    
    float64[6] xd                # task pos(posx)  
    float64[6] xc                # task pos(posx)  
    int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOrientationErrorRequest(null);
    if (msg.xd !== undefined) {
      resolved.xd = msg.xd;
    }
    else {
      resolved.xd = new Array(6).fill(0)
    }

    if (msg.xc !== undefined) {
      resolved.xc = msg.xc;
    }
    else {
      resolved.xc = new Array(6).fill(0)
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = 0
    }

    return resolved;
    }
};

class GetOrientationErrorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ori_error = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('ori_error')) {
        this.ori_error = initObj.ori_error
      }
      else {
        this.ori_error = 0.0;
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
    // Serializes a message object of type GetOrientationErrorResponse
    // Serialize message field [ori_error]
    bufferOffset = _serializer.float32(obj.ori_error, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOrientationErrorResponse
    let len;
    let data = new GetOrientationErrorResponse(null);
    // Deserialize message field [ori_error]
    data.ori_error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetOrientationErrorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7320195725fdc25f09498194a9fd93cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32    ori_error         # orientation error  
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOrientationErrorResponse(null);
    if (msg.ori_error !== undefined) {
      resolved.ori_error = msg.ori_error;
    }
    else {
      resolved.ori_error = 0.0
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
  Request: GetOrientationErrorRequest,
  Response: GetOrientationErrorResponse,
  md5sum() { return '83afbc83b0df709d838d4a917c3c9c05'; },
  datatype() { return 'dsr_msgs/GetOrientationError'; }
};
