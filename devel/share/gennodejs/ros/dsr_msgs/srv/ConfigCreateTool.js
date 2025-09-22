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

class ConfigCreateToolRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.weight = null;
      this.cog = null;
      this.inertia = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = 0.0;
      }
      if (initObj.hasOwnProperty('cog')) {
        this.cog = initObj.cog
      }
      else {
        this.cog = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('inertia')) {
        this.inertia = initObj.inertia
      }
      else {
        this.inertia = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigCreateToolRequest
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [weight]
    bufferOffset = _serializer.float64(obj.weight, buffer, bufferOffset);
    // Check that the constant length array field [cog] has the right length
    if (obj.cog.length !== 3) {
      throw new Error('Unable to serialize array field cog - length must be 3')
    }
    // Serialize message field [cog]
    bufferOffset = _arraySerializer.float64(obj.cog, buffer, bufferOffset, 3);
    // Check that the constant length array field [inertia] has the right length
    if (obj.inertia.length !== 6) {
      throw new Error('Unable to serialize array field inertia - length must be 6')
    }
    // Serialize message field [inertia]
    bufferOffset = _arraySerializer.float64(obj.inertia, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigCreateToolRequest
    let len;
    let data = new ConfigCreateToolRequest(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [weight]
    data.weight = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cog]
    data.cog = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [inertia]
    data.inertia = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    return length + 84;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ConfigCreateToolRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84aa36be3ab38a7a2e68e39b9044198f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # config_create_tool 
    # It is a service for registering and using robot Tool information in advance for safety 
    #____________________________________________________________________________________________
    
    string          name        # tool name 
    float64         weight      # tool weight 
    float64[3]      cog         # Center of gravity
    float64[6]      inertia     # tool inertia 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigCreateToolRequest(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.weight !== undefined) {
      resolved.weight = msg.weight;
    }
    else {
      resolved.weight = 0.0
    }

    if (msg.cog !== undefined) {
      resolved.cog = msg.cog;
    }
    else {
      resolved.cog = new Array(3).fill(0)
    }

    if (msg.inertia !== undefined) {
      resolved.inertia = msg.inertia;
    }
    else {
      resolved.inertia = new Array(6).fill(0)
    }

    return resolved;
    }
};

class ConfigCreateToolResponse {
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
    // Serializes a message object of type ConfigCreateToolResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigCreateToolResponse
    let len;
    let data = new ConfigCreateToolResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ConfigCreateToolResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigCreateToolResponse(null);
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
  Request: ConfigCreateToolRequest,
  Response: ConfigCreateToolResponse,
  md5sum() { return 'd748bfa923d2a13eaf013a95f67bd0ec'; },
  datatype() { return 'dsr_msgs/ConfigCreateTool'; }
};
