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

class GetUserCartCoordRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetUserCartCoordRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int8(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUserCartCoordRequest
    let len;
    let data = new GetUserCartCoordRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetUserCartCoordRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d504299943ad968aabe3de24053d208';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # posx, ref = get_user_cart_coord(id)   
    #____________________________________________________________________________________________
    # This service is only available in M2.50 or later
    
    int8       id                # ID of user coord 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUserCartCoordRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

class GetUserCartCoordResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.conv_posx = null;
      this.ref = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('conv_posx')) {
        this.conv_posx = initObj.conv_posx
      }
      else {
        this.conv_posx = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
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
    // Serializes a message object of type GetUserCartCoordResponse
    // Check that the constant length array field [conv_posx] has the right length
    if (obj.conv_posx.length !== 6) {
      throw new Error('Unable to serialize array field conv_posx - length must be 6')
    }
    // Serialize message field [conv_posx]
    bufferOffset = _arraySerializer.float64(obj.conv_posx, buffer, bufferOffset, 6);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUserCartCoordResponse
    let len;
    let data = new GetUserCartCoordResponse(null);
    // Deserialize message field [conv_posx]
    data.conv_posx = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 50;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetUserCartCoordResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'abfa434bf46c1e9ef44fb450528140a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] conv_posx         # task pos(posx)  
    int8       ref               # Reference coordinate of the coordinate to get
    bool       success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUserCartCoordResponse(null);
    if (msg.conv_posx !== undefined) {
      resolved.conv_posx = msg.conv_posx;
    }
    else {
      resolved.conv_posx = new Array(6).fill(0)
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
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
  Request: GetUserCartCoordRequest,
  Response: GetUserCartCoordResponse,
  md5sum() { return 'a933e25ab5287ae9b6e7ebd1be20a647'; },
  datatype() { return 'dsr_msgs/GetUserCartCoord'; }
};
