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

class GetCtrlBoxDigitalInputRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCtrlBoxDigitalInputRequest
    // Serialize message field [index]
    bufferOffset = _serializer.int8(obj.index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCtrlBoxDigitalInputRequest
    let len;
    let data = new GetCtrlBoxDigitalInputRequest(null);
    // Deserialize message field [index]
    data.index = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCtrlBoxDigitalInputRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4ecff52046eaecf0190e650b1958286';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_digital_input
    # This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.
    #____________________________________________________________________________________________
    
    int8        index    # Digital Input in Control Box(1 ~ 16) 
                         # <GPIO_CTRLBOX_DIGITAL_INDEX>
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCtrlBoxDigitalInputRequest(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    return resolved;
    }
};

class GetCtrlBoxDigitalInputResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
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
    // Serializes a message object of type GetCtrlBoxDigitalInputResponse
    // Serialize message field [value]
    bufferOffset = _serializer.int8(obj.value, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCtrlBoxDigitalInputResponse
    let len;
    let data = new GetCtrlBoxDigitalInputResponse(null);
    // Deserialize message field [value]
    data.value = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCtrlBoxDigitalInputResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2002a87d9e124a06af2057212386843f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8        value    # 0=OFF, 1=ON
    bool        success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCtrlBoxDigitalInputResponse(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
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
  Request: GetCtrlBoxDigitalInputRequest,
  Response: GetCtrlBoxDigitalInputResponse,
  md5sum() { return '66249e10e881902f2908d5046d836788'; },
  datatype() { return 'dsr_msgs/GetCtrlBoxDigitalInput'; }
};
