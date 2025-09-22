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

class GetToolForceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetToolForceRequest
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetToolForceRequest
    let len;
    let data = new GetToolForceRequest(null);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetToolForceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d7349a71916d8843a1403dfc21af071';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_tool_force(ref)
    # returns the external force applied to the current tool
    #____________________________________________________________________________________________
    # This service returns the external force applied to the current tool based on the ref coordinate. 
    # The force is based on the base coordinate while the moment is based on the tool coordinate.
    
    int8       ref               # DR_BASE(0), DR_WORLD(2)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetToolForceRequest(null);
    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    return resolved;
    }
};

class GetToolForceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tool_force = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('tool_force')) {
        this.tool_force = initObj.tool_force
      }
      else {
        this.tool_force = new Array(6).fill(0);
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
    // Serializes a message object of type GetToolForceResponse
    // Check that the constant length array field [tool_force] has the right length
    if (obj.tool_force.length !== 6) {
      throw new Error('Unable to serialize array field tool_force - length must be 6')
    }
    // Serialize message field [tool_force]
    bufferOffset = _arraySerializer.float64(obj.tool_force, buffer, bufferOffset, 6);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetToolForceResponse
    let len;
    let data = new GetToolForceResponse(null);
    // Deserialize message field [tool_force]
    data.tool_force = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetToolForceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83ad17c3584df6e899fee8fe64b1d4dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] tool_force        # External force applied to the tool
    bool       success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetToolForceResponse(null);
    if (msg.tool_force !== undefined) {
      resolved.tool_force = msg.tool_force;
    }
    else {
      resolved.tool_force = new Array(6).fill(0)
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
  Request: GetToolForceRequest,
  Response: GetToolForceResponse,
  md5sum() { return '9590f477df24281be1fec298a858c77f'; },
  datatype() { return 'dsr_msgs/GetToolForce'; }
};
