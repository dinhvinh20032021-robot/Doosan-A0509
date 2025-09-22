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

class GetDrlStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDrlStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDrlStateRequest
    let len;
    let data = new GetDrlStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetDrlStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_drl_state
    # Get DRL Program State
    # 0 : DRL_PROGRAM_STATE_PLAY
    # 1 : DRL_PROGRAM_STATE_STOP
    # 2 : DRL_PROGRAM_STATE_HOLD
    # 3 : DRL_PROGRAM_STATE_LAST
    # drfl.GetProgramState()
    #____________________________________________________________________________________________
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDrlStateRequest(null);
    return resolved;
    }
};

class GetDrlStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drl_state = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('drl_state')) {
        this.drl_state = initObj.drl_state
      }
      else {
        this.drl_state = 0;
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
    // Serializes a message object of type GetDrlStateResponse
    // Serialize message field [drl_state]
    bufferOffset = _serializer.int8(obj.drl_state, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDrlStateResponse
    let len;
    let data = new GetDrlStateResponse(null);
    // Deserialize message field [drl_state]
    data.drl_state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetDrlStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9dd00c5d3b26eace3288075b31af3114';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8        drl_state # <DRL_PROGRAM_STATE>
    bool        success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDrlStateResponse(null);
    if (msg.drl_state !== undefined) {
      resolved.drl_state = msg.drl_state;
    }
    else {
      resolved.drl_state = 0
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
  Request: GetDrlStateRequest,
  Response: GetDrlStateResponse,
  md5sum() { return '9dd00c5d3b26eace3288075b31af3114'; },
  datatype() { return 'dsr_msgs/GetDrlState'; }
};
